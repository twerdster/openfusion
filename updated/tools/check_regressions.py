#!/usr/bin/env python3
"""Build isolated defect variants and require our GPU tests to reject them.

Requires a native Linux filesystem, CMake/CUDA and a GPU. Original source is read
only. Build failures/timeouts are errors, never evidence of a detected regression.
The ICP variant initializes shared scratch with finite canaries to expose reads
of entries the original 256-thread block never initializes. That is deliberate
fault instrumentation, not a claim about an unmodified kernel's memory contents.
"""
import argparse
import json
import pathlib
import shutil
import subprocess


def run(command, cwd, log, timeout=180):
    with log.open('w') as stream:
        result = subprocess.run(command, cwd=cwd, stdout=stream,
                                stderr=subprocess.STDOUT, timeout=timeout)
    return result.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work-dir', required=True, type=pathlib.Path)
    args = parser.parse_args()
    root = pathlib.Path(__file__).resolve().parents[2]
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    results = []
    for variant in ('original-integration', 'original-icp-poisoned-scratch',
                    'fixed-icp-poisoned-scratch'):
        target = work / variant
        if target.exists():
            raise RuntimeError(f'Refusing to overwrite existing variant: {target}')
        target.mkdir()
        (target / 'source').symlink_to(root / 'source', target_is_directory=True)
        dest = target / 'updated'
        dest.mkdir()
        for name in ('src', 'include', 'compat', 'tools'):
            shutil.copytree(root / 'updated' / name, dest / name,
                            ignore=shutil.ignore_patterns('__pycache__'))
        shutil.copy2(root / 'updated/CMakeLists.txt', dest / 'CMakeLists.txt')
        if variant == 'original-integration':
            shutil.copy2(root / 'source/Source/src/VolumeIntegrator.cu',
                         dest / 'src/VolumeIntegrator.cu')
            expected = ('guard slab', 'TSDF integration weight differs',
                        'TSDF integration differs')
        else:
            fixed_control = variant == 'fixed-icp-poisoned-scratch'
            original = ((dest / 'src/ICP.cu').read_text() if fixed_control
                        else (root / 'source/Source/src/ICP.cu').read_text())
            marker = '\tfloat A_row[NUM_VARS]'
            assert marker in original
            canaries = '''
    // Ablation-only instrumentation. Correct reductions must not consume the
    // unused upper half. A finite value models arbitrary prior scratch data.
    const unsigned scratchLane = blockDim.x * threadIdx.y + threadIdx.x;
    for (unsigned q = scratchLane; q < CORRESPONDENCE_BLOCK;
         q += blockDim.x * blockDim.y) {
        s_AtA[q] = s_Atb[q] = 1.0f;
    }
    __syncthreads();
'''
            original = original.replace(marker, canaries + marker, 1)
            port = (dest / 'src/ICP.cu').read_text()
            start = port.index('void computeCorrespondenceForTest(')
            stop = port.index('\n\nvoid ICP::FindCorresponding', start)
            (dest / 'src/ICP.cu').write_text(original if fixed_control
                else original + '\n' + port[start:stop] + '\n')
            if not fixed_control:
                header = dest / 'include/Constants.h'
                header.write_text(header.read_text().replace(
                    '#define CORRESPONDENCE_BLOCK (BLOCK_SIZE * BLOCK_SIZE)',
                    '#define CORRESPONDENCE_BLOCK (512)'))
            expected = ('ICP AtA differs', 'ICP Atb differs')
        commands = [
            ['cmake', '-S', 'updated', '-B', 'build',
             '-DCMAKE_BUILD_TYPE=RelWithDebInfo', '-DOPENFUSION_VOLUME_SIZE=128'],
            ['cmake', '--build', 'build', '-j2'],
        ]
        for i, command in enumerate(commands):
            if run(command, target, target / f'build-{i}.log'):
                raise RuntimeError(f'{variant}: build failed; see {target}')
        code = run(['build/openfusion_headless', '--self-test', '--output',
                    'test-output'], target, target / 'test.log')
        log = (target / 'test.log').read_text()
        expected_success = variant == 'fixed-icp-poisoned-scratch'
        detected = (code == 0 if expected_success else
                    code != 0 and any(message in log for message in expected))
        results.append({'variant': variant, 'test_exit_code': code,
                        'expectation_met': detected,
                        'expected_success': expected_success,
                        'instrumentation': ('none' if variant == 'original-integration'
                            else 'shared scratch initialized to finite 1.0 canaries'),
                        'failure': next((line for line in log.splitlines()
                                         if line.startswith('FAIL:')), '')})
        print(json.dumps(results[-1]), flush=True)
        if not detected:
            raise RuntimeError(f'{variant}: expected numerical/guard failure absent')
    (work / 'results.json').write_text(json.dumps(results, indent=2) + '\n')


if __name__ == '__main__':
    main()

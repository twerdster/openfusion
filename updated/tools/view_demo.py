#!/usr/bin/env python3
"""Make a self-contained viewer from OpenFusion's saved PPM/PLY/JSON output.

Uses only the Python standard library. It does not run reconstruction or alter
the saved measurements. The point cloud is displayed in the PLY coordinates.
"""

import argparse
import base64
import html
import json
import math
from pathlib import Path
import re
import struct
import zlib


def ppm_png(path):
    """Convert an 8-bit binary PPM (P6) into an in-memory PNG."""
    data = path.read_bytes()
    cursor = 0

    def token():
        nonlocal cursor
        while cursor < len(data):
            if data[cursor:cursor + 1] == b"#":
                end = data.find(b"\n", cursor)
                cursor = len(data) if end < 0 else end + 1
            elif data[cursor] in b" \t\r\n":
                cursor += 1
            else:
                break
        start = cursor
        while cursor < len(data) and data[cursor] not in b" \t\r\n":
            cursor += 1
        return data[start:cursor]

    if token() != b"P6":
        raise ValueError(f"{path}: expected a binary P6 PPM")
    width, height, maximum = int(token()), int(token()), int(token())
    if width <= 0 or height <= 0 or maximum != 255:
        raise ValueError(f"{path}: expected positive dimensions and maxval 255")
    if cursor >= len(data) or data[cursor] not in b" \t\r\n":
        raise ValueError(f"{path}: missing PPM header separator")
    cursor += 2 if data[cursor:cursor + 2] == b"\r\n" else 1
    pixels = data[cursor:]
    if len(pixels) != width * height * 3:
        raise ValueError(f"{path}: truncated PPM or unexpected trailing bytes")

    def chunk(name, payload):
        return (struct.pack(">I", len(payload)) + name + payload +
                struct.pack(">I", zlib.crc32(name + payload)))

    rows = b"".join(b"\0" + pixels[row * width * 3:(row + 1) * width * 3]
                    for row in range(height))
    return (b"\x89PNG\r\n\x1a\n" +
            chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
            chunk(b"IDAT", zlib.compress(rows, 9)) + chunk(b"IEND", b""))


def ply_points(path, limit):
    """Read finite vertices, using a deterministic evenly spaced display sample."""
    with path.open(encoding="ascii") as stream:
        if stream.readline().strip() != "ply":
            raise ValueError(f"{path}: missing PLY signature")
        count, properties, in_vertices, ascii_format = None, [], False, False
        for line in stream:
            fields = line.split()
            if fields[:2] == ["format", "ascii"]:
                ascii_format = True
            elif fields[:1] == ["element"]:
                in_vertices = fields[1] == "vertex"
                if in_vertices:
                    count = int(fields[2])
            elif fields[:1] == ["property"] and in_vertices:
                if fields[1] == "list":
                    raise ValueError(f"{path}: vertex list properties are unsupported")
                properties.append(fields[-1])
            elif fields[:1] == ["end_header"]:
                break
        if not ascii_format or count is None:
            raise ValueError(f"{path}: expected ASCII PLY with a vertex element")
        indices = [properties.index(axis) for axis in ("x", "y", "z")]
        color_indices = ([properties.index(name) for name in ("red", "green", "blue")]
                         if all(name in properties for name in ("red", "green", "blue"))
                         else None)
        stride = max(1, math.ceil(count / limit))
        points, colors = [], []
        for index in range(count):
            fields = stream.readline().split()
            if len(fields) < len(properties):
                raise ValueError(f"{path}: truncated vertex {index}")
            point = [float(fields[position]) for position in indices]
            if not all(math.isfinite(value) for value in point):
                raise ValueError(f"{path}: nonfinite vertex {index}")
            if index % stride == 0:
                points.append(point)
                if color_indices:
                    colors.append([max(0, min(255, int(fields[position])))
                                   for position in color_indices])
        return {"name": path.name, "vertex_count": count,
                "points": points, "colors": colors}


def build_report(directory, destination, point_limit, input_label, units):
    images = []
    frames = []
    for path in sorted(directory.glob("*.ppm")):
        encoded = base64.b64encode(ppm_png(path)).decode("ascii")
        if re.fullmatch(r"reconstruction-\d+\.ppm", path.name):
            frames.append({"name": path.name, "image": "data:image/png;base64," + encoded})
            continue
        images.append(f'<figure><img alt="{html.escape(path.stem)}" '
                      f'src="data:image/png;base64,{encoded}"><figcaption>'
                      f'{html.escape(path.name)}</figcaption></figure>')
    clouds = [ply_points(path, point_limit) for path in sorted(directory.glob("*.ply"))]
    records = []
    measurements = []
    recorded_input = None
    metric_note = "See the full JSON record for recorded scope, operators and per-frame output."
    for path in sorted(directory.glob("*.json")):
        value = json.loads(path.read_text(encoding="utf-8"))
        if path.name == "metrics.json" and isinstance(value, dict):
            recorded_input = value.get("input")
            if value.get("ground_truth_available") is False:
                metric_note = ("This replay has no independent ground truth. Recorded status indicates "
                               "operator execution and finite output, not measured reconstruction or pose accuracy.")
            elif "final_raycast_surface_rmse_mm" in value:
                metric_note = ("Values below are copied from metrics.json. Surface error is distance to an "
                               "analytic scene surface; same-ray range error compares visible depth along an "
                               "image ray and also captures silhouette and visibility mismatches. See the full "
                               "record for per-frame values and the operator chain.")
            labels = {
                "status": "Recorded status",
                "ground_truth_available": "Independent ground truth available",
                "volume_size": "Voxels per axis",
                "voxel_width_mm": "Voxel width (mm)",
                "maximum_translation_error_mm": "Maximum pose translation error (mm)",
                "maximum_rotation_error_degrees": "Maximum pose rotation error (degrees)",
                "final_raycast_surface_rmse_mm": "Final raycast surface RMSE (mm)",
                "final_raycast_ray_distance_rmse_mm": "Final same-ray range RMSE (mm)",
                "stationary_pose_final_translation_error_mm": "Stationary-pose baseline final translation error (mm)",
                "estimated_camera_displacement_mm": "Estimated camera displacement (mm)",
                "final_raycast_hits": "Final raycast surface samples",
                "observed_voxels": "Observed voxels",
                "maximum_voxel_weight": "Maximum voxel weight",
                "guard_slabs_unchanged": "Allocation guard slabs unchanged",
            }
            for key, label in labels.items():
                if key in value:
                    number = value[key]
                    display = (number if isinstance(number, str) else
                               f"{number:.6g}" if isinstance(number, float) else json.dumps(number))
                    measurements.append(f"<tr><th>{html.escape(label)}</th>"
                                        f"<td>{html.escape(display)}</td></tr>")
        records.append(f'<details><summary>{html.escape(path.name)} (full record)</summary>'
                       f'<pre>{html.escape(json.dumps(value, indent=2, ensure_ascii=False))}'
                       '</pre></details>')
    if not images and not frames and not clouds and not records:
        raise ValueError(f"{directory}: no PPM, ASCII PLY, or JSON output found")
    # Escape less-than characters so a filename cannot end the script element.
    cloud_json = json.dumps(clouds, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    frame_json = json.dumps(frames, separators=(",", ":")).replace("<", "\\u003c")
    document = TEMPLATE.replace("__IMAGES__", "\n".join(images))
    document = document.replace("__RECORDS__", "\n".join(records))
    document = document.replace("__MEASUREMENTS__", "\n".join(measurements))
    document = document.replace("__INPUT__", html.escape(input_label or recorded_input or
                                                        "saved output; input provenance not recorded"))
    document = document.replace("__METRIC_NOTE__", html.escape(metric_note))
    document = document.replace("__UNITS__", html.escape(units))
    document = document.replace("__CLOUDS__", cloud_json)
    document = document.replace("__FRAMES__", frame_json)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(document, encoding="utf-8")


TEMPLATE = '''<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenFusion saved execution</title>
<style>
:root{color-scheme:dark;font:16px system-ui,sans-serif;background:#111820;color:#e7eef5}
body{max-width:1180px;margin:32px auto;padding:0 20px}h1{font-size:1.8rem}p{line-height:1.6;max-width:90ch}
.images{display:flex;flex-wrap:wrap;gap:20px}figure{margin:0;background:#1b2733;border:1px solid #3a4b5e;padding:12px;flex:1 1 260px}
img{width:100%;height:auto;image-rendering:pixelated}figcaption{margin-top:8px;color:#b8cbdd;font-family:monospace}
canvas{display:block;width:100%;height:520px;background:#080d13;touch-action:none;border:1px solid #3a4b5e}
.controls{display:flex;align-items:center;flex-wrap:wrap;gap:16px;margin:14px 0}button,select{font:inherit;color:inherit;background:#253544;padding:6px 10px;border:1px solid #60738a;border-radius:4px}
pre{overflow:auto;padding:16px;background:#18232e;line-height:1.5;font-size:13px}details{margin:18px 0}summary{cursor:pointer;color:#b8cbdd}footer{margin:30px 0;color:#b8cbdd}
table{border-collapse:collapse;width:100%;max-width:800px}th,td{text-align:left;padding:9px 12px;border-bottom:1px solid #3a4b5e}th{font-weight:500}td{font-family:monospace}
</style>
<h1>OpenFusion saved execution</h1>
<p>Input: <strong>__INPUT__</strong>. The images, point clouds and records below come from files written by the updated headless runner. This viewer does not reconstruct a scene or recompute the recorded measurements. Historical project figures are not included.</p>
<div class="images">__IMAGES__</div>
<section id="frame-section"><h2>Saved reconstruction sequence</h2>
<p>Each frame below is a saved raycast from the runner, in file order. Playback changes the displayed image only; it does not execute OpenFusion.</p>
<div class="controls"><button id="play" type="button">Play saved frames</button><label>Frame <input id="frame-slider" type="range" min="0" value="0"></label><span id="frame-name"></span></div>
<figure><img id="frame-image" alt="Saved reconstruction frame" style="max-width:640px"></figure></section>
<section id="cloud-section"><h2>Saved surface points</h2>
<p>Drag to rotate; use the wheel to zoom. Coordinates are in <strong>__UNITS__</strong>. Display sampling affects only this view; the saved PLY retains all exported vertices. These are raycast surface samples, not a triangulated mesh.</p>
<div class="controls"><label>Point cloud <select id="cloud-select"></select></label><button id="reset" type="button">Reset view</button><span id="point-count"></span></div>
<canvas id="cloud" aria-label="Interactive view of the saved surface point cloud"></canvas>
</section><h2>Recorded output</h2>
<p>__METRIC_NOTE__</p>
<table>__MEASUREMENTS__</table>__RECORDS__
<footer>Self-contained report generated by updated/tools/view_demo.py. No external scripts, network requests or browser extensions are required.</footer>
<script>
"use strict";
const clouds=__CLOUDS__;
const frames=__FRAMES__,slider=document.getElementById('frame-slider');
let playback=null;
function showFrame(){const frame=frames[Number(slider.value)];if(!frame)return;document.getElementById('frame-image').src=frame.image;document.getElementById('frame-name').textContent=frame.name}
if(!frames.length){document.getElementById('frame-section').hidden=true}else{slider.max=frames.length-1;showFrame()}
slider.addEventListener('input',showFrame);
document.getElementById('play').addEventListener('click',()=>{if(playback){clearInterval(playback);playback=null;document.getElementById('play').textContent='Play saved frames'}else{playback=setInterval(()=>{slider.value=(Number(slider.value)+1)%frames.length;showFrame()},250);document.getElementById('play').textContent='Pause'}});
const canvas=document.getElementById('cloud'),ctx=canvas.getContext('2d'),selector=document.getElementById('cloud-select');
let chosen=0,yaw=0,pitch=0,zoom=1,drag=null;
clouds.forEach((cloud,index)=>{const option=document.createElement('option');option.value=index;option.textContent=cloud.name;selector.append(option)});
if(!clouds.length){document.getElementById('cloud-section').hidden=true}
function draw(){
 if(!clouds.length)return;
 const cloud=clouds[chosen],points=cloud.points,rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;
 canvas.width=Math.round(rect.width*ratio);canvas.height=Math.round(rect.height*ratio);ctx.setTransform(ratio,0,0,ratio,0,0);
 ctx.fillStyle='#080d13';ctx.fillRect(0,0,rect.width,rect.height);
 document.getElementById('point-count').textContent=points.length.toLocaleString()+' displayed / '+cloud.vertex_count.toLocaleString()+' exported points';
 if(!points.length)return;
 const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];
 for(const point of points)for(let axis=0;axis<3;axis++){lo[axis]=Math.min(lo[axis],point[axis]);hi[axis]=Math.max(hi[axis],point[axis])}
 const center=lo.map((v,i)=>(v+hi[i])/2),extent=Math.max(...hi.map((v,i)=>v-lo[i]),1),scale=Math.min(rect.width,rect.height)*0.76/extent*zoom;
 const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),transformed=[];
 for(let i=0;i<points.length;i++){
  const p=points[i],x=p[0]-center[0],y=p[1]-center[1],z=p[2]-center[2],rx=cy*x+sy*z,rz=-sy*x+cy*z,ry=cp*y-sp*rz,depth=sp*y+cp*rz;
  transformed.push([rx*scale+rect.width/2,-ry*scale+rect.height/2,depth,i]);
 }
 transformed.sort((a,b)=>a[2]-b[2]);
 for(const p of transformed){
  const color=cloud.colors[p[3]];
  if(color){ctx.fillStyle='rgb('+color.join(',')+')'}else{const t=Math.max(0,Math.min(1,p[2]/extent+0.5));ctx.fillStyle='hsl('+(190-35*t)+' 58% '+(44+24*t)+'%)'}
  ctx.fillRect(p[0],p[1],2,2);
 }
 ctx.fillStyle='#c4d5e5';ctx.font='12px system-ui';ctx.fillText('Orthographic point-cloud view; centered for display',12,22);
}
selector.addEventListener('change',()=>{chosen=Number(selector.value);yaw=pitch=0;zoom=1;draw()});
document.getElementById('reset').addEventListener('click',()=>{yaw=pitch=0;zoom=1;draw()});
canvas.addEventListener('pointerdown',event=>{canvas.setPointerCapture(event.pointerId);drag=[event.clientX,event.clientY]});
canvas.addEventListener('pointermove',event=>{if(!drag)return;yaw+=(event.clientX-drag[0])*0.01;pitch+=(event.clientY-drag[1])*0.01;drag=[event.clientX,event.clientY];draw()});
canvas.addEventListener('pointerup',()=>{drag=null});canvas.addEventListener('pointercancel',()=>{drag=null});
canvas.addEventListener('wheel',event=>{event.preventDefault();zoom=Math.max(0.1,Math.min(10,zoom*Math.exp(-event.deltaY*0.001)));draw()},{passive:false});
window.addEventListener('resize',draw);draw();
</script></html>
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="runner output directory")
    parser.add_argument("--output", type=Path, help="destination HTML (default: directory/demo.html)")
    parser.add_argument("--max-points", type=int, default=60000, help="display sample limit per cloud")
    parser.add_argument("--input-label", help="override input label (default: metrics.json input field)")
    parser.add_argument("--units", default="millimetres")
    args = parser.parse_args()
    if args.max_points < 1:
        parser.error("--max-points must be positive")
    try:
        output = args.output or args.directory / "demo.html"
        build_report(args.directory, output, args.max_points, args.input_label, args.units)
    except (OSError, ValueError, KeyError, IndexError) as error:
        parser.exit(1, f"{error}\n")
    print(output)


if __name__ == "__main__":
    main()

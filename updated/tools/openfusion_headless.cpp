// Updated maintenance harness. The reconstruction operators are the recovered
// OpenFusion CUDA operators, linked from updated/src, not a CPU substitute.
#include "CameraFromFile.h"
#include "CameraHandler.h"
#include "General.h"
#include "ICP.h"
#include "Raycaster.h"
#include "VolumeIntegrator.h"
#include <cuda_runtime.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

void computeCorrespondenceForTest(const CameraData&, const PointInfo&, float*,
                                 float*, const float*, const CameraParams&);

namespace fs = std::filesystem;
namespace {
void check(cudaError_t status, const char* operation) {
    if (status != cudaSuccess)
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
}
void sync() { check(cudaGetLastError(), "CUDA kernel launch"); check(cudaDeviceSynchronize(), "CUDA synchronize"); }
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
template<class T> struct Device {
    T* p = nullptr;
    size_t n;
    explicit Device(size_t count) : n(count) { check(cudaMalloc(reinterpret_cast<void**>(&p), n*sizeof(T)), "cudaMalloc"); }
    ~Device() { if (p) cudaFree(p); }
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;
    void put(const std::vector<T>& values) {
        require(values.size() == n, "device upload length");
        check(cudaMemcpy(p, values.data(), n*sizeof(T), cudaMemcpyHostToDevice), "upload");
    }
    std::vector<T> get() const {
        std::vector<T> result(n);
        check(cudaMemcpy(result.data(), p, n*sizeof(T), cudaMemcpyDeviceToHost), "download");
        return result;
    }
};
float3 vec(float x, float y, float z) { return make_float3(x,y,z); }
float3 add(float3 a, float3 b) { return vec(a.x+b.x,a.y+b.y,a.z+b.z); }
float3 sub(float3 a, float3 b) { return vec(a.x-b.x,a.y-b.y,a.z-b.z); }
float3 scale(float3 a, float s) { return vec(a.x*s,a.y*s,a.z*s); }
float dot3(float3 a, float3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
float norm(float3 a) { return std::sqrt(dot3(a,a)); }
float3 cross3(float3 a,float3 b) { return vec(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x); }
float3 unit(float3 a) { float l=norm(a); return l>0 ? scale(a,1/l) : vec(0,0,0); }
bool valid(float3 a) { return a.x!=BAD_VERTEX && a.y!=BAD_VERTEX && a.z!=BAD_VERTEX && std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }
using Pose = std::array<float,16>;
Pose pose(float tx=0, float ty=0, float tz=CAMERA_START_Z_MM, float yaw=0) {
    float c=std::cos(yaw),s=std::sin(yaw);
    return {c,0,s,tx, 0,1,0,ty, -s,0,c,tz, 0,0,0,1};
}
float3 transform(const Pose& t,float3 v) {
    return vec(t[0]*v.x+t[1]*v.y+t[2]*v.z+t[3], t[4]*v.x+t[5]*v.y+t[6]*v.z+t[7], t[8]*v.x+t[9]*v.y+t[10]*v.z+t[11]);
}
Pose inverse(const Pose& t) {
    Pose r={t[0],t[4],t[8],0, t[1],t[5],t[9],0, t[2],t[6],t[10],0, 0,0,0,1};
    float3 v=transform(r,vec(-t[3],-t[7],-t[11])); r[3]=v.x;r[7]=v.y;r[11]=v.z;return r;
}
float translationError(const Pose& a,const Pose& b) { return norm(vec(a[3]-b[3],a[7]-b[7],a[11]-b[11])); }
float rotationError(const Pose& a,const Pose& b) {
    double trace=0; for(int i=0;i<3;++i)for(int j=0;j<3;++j)trace+=double(a[i*4+j])*b[i*4+j];
    return float(std::acos(std::clamp((trace-1)/2,-1.0,1.0))*180/MATH_PI);
}
CameraParams parameters(unsigned width=97,unsigned height=73) {
    CameraParams p{}; p.m_width=width;p.m_height=height;
    p.m_intrinsic.Set(-float(width)*1.05f,-float(width)*1.05f,(width-1)*.5f,(height-1)*.5f);
    p.m_invIntrinsic.Set(1/p.m_intrinsic.m_fx,1/p.m_intrinsic.m_fy,-p.m_intrinsic.m_cx/p.m_intrinsic.m_fx,-p.m_intrinsic.m_cy/p.m_intrinsic.m_fy);
    p.m_min_depth=100;p.m_max_depth=2000;return p;
}
float3 unproject(const CameraParams& p,unsigned x,unsigned y,float depth) {
    return vec(-depth*(p.m_invIntrinsic.m_fx*x+p.m_invIntrinsic.m_cx),-depth*(p.m_invIntrinsic.m_fy*y+p.m_invIntrinsic.m_cy),-depth);
}
struct HostMap { std::vector<CAMERA_DEPTH_TYPE> depth; std::vector<float3> vertices,normals; };
HostMap referenceMap(const CameraParams& p,const std::vector<CAMERA_DEPTH_TYPE>& depth,const Pose& t) {
    HostMap m{depth,std::vector<float3>(depth.size()),std::vector<float3>(depth.size(),vec(0,0,0))};
    for(unsigned y=0;y<p.m_height;++y)for(unsigned x=0;x<p.m_width;++x) {
        size_t i=y*p.m_width+x;
        m.vertices[i]=(depth[i]<p.m_min_depth||depth[i]>p.m_max_depth)?vec(BAD_VERTEX,BAD_VERTEX,BAD_VERTEX):transform(t,unproject(p,x,y,depth[i]));
    }
    for(unsigned y=0;y+1<p.m_height;++y)for(unsigned x=0;x+1<p.m_width;++x) {
        size_t i=y*p.m_width+x;
        if(valid(m.vertices[i])&&valid(m.vertices[i+1])&&valid(m.vertices[i+p.m_width]))
            m.normals[i]=unit(cross3(sub(m.vertices[i+1],m.vertices[i]),sub(m.vertices[i+p.m_width],m.vertices[i])));
    } return m;
}
struct Sphere { float3 center; float radius; };
const std::array<Sphere,3> spheres={Sphere{vec(-115,-55,30),118},Sphere{vec(122,65,-55),98},Sphere{vec(15,-155,-135),80}};
// Returns positive camera-axis depth, in millimeters. The fixture is raytraced
// analytically; this CPU code generates input and independent geometric truth.
float sceneDepth(const CameraParams& p,unsigned x,unsigned y,const Pose& t,bool plane=false) {
    float3 origin=vec(t[3],t[7],t[11]);
    float3 direction=sub(transform(t,unproject(p,x,y,1)),origin);
    float closest=std::numeric_limits<float>::infinity();
    if(std::abs(direction.z)>1e-9f) {
        float hit=((plane?0.f:-300.f)-origin.z)/direction.z;
        if(hit>0)closest=hit;
    }
    if(!plane)for(const auto& s:spheres) {
        float3 o=sub(origin,s.center); float a=dot3(direction,direction),b=2*dot3(o,direction),c=dot3(o,o)-s.radius*s.radius;
        float discriminant=b*b-4*a*c;
        if(discriminant>=0) { float hit=(-b-std::sqrt(discriminant))/(2*a);if(hit>0)closest=std::min(closest,hit); }
    }
    return closest;
}
float sceneSurfaceDistance(float3 v) {
    float distance=std::abs(v.z+300.f);
    for(const auto& s:spheres)distance=std::min(distance,std::abs(norm(sub(v,s.center))-s.radius));
    return distance;
}
float percentile(std::vector<float> values,float fraction) {
    if(values.empty())return std::numeric_limits<float>::infinity();
    size_t index=std::min(values.size()-1,size_t(std::ceil(values.size()*fraction)-1));
    std::nth_element(values.begin(),values.begin()+index,values.end());return values[index];
}
std::vector<CAMERA_DEPTH_TYPE> scene(const CameraParams& p,const Pose& t,bool plane=false) {
    std::vector<CAMERA_DEPTH_TYPE> result(p.m_width*p.m_height);
    for(unsigned y=0;y<p.m_height;++y)for(unsigned x=0;x<p.m_width;++x) {
        float z=sceneDepth(p,x,y,t,plane);
        if(std::isfinite(z)&&z>=p.m_min_depth&&z<=p.m_max_depth)result[y*p.m_width+x]=static_cast<CAMERA_DEPTH_TYPE>(std::lround(z));
    }return result;
}
void writeRecording(const fs::path& dir,const CameraParams& p,const std::vector<std::vector<CAMERA_DEPTH_TYPE>>& frames) {
    fs::create_directories(dir);
    std::ofstream f(dir/"Parameters.txt");
    f<<std::setprecision(9)<<p.m_width<<' '<<p.m_height<<'\n'<<p.m_intrinsic.m_fx<<' '<<p.m_intrinsic.m_fy<<' '<<p.m_intrinsic.m_cx<<' '<<p.m_intrinsic.m_cy<<"\n0\n"<<p.m_min_depth<<'\n'<<p.m_max_depth<<'\n'<<frames.size()<<"\nSynthetic analytic spheres fixture\n";
    require(bool(f),"write calibration");
    for(size_t i=0;i<frames.size();++i) {
        std::ofstream d(dir/("recorded"+std::to_string(i+1)),std::ios::binary);
        d.write(reinterpret_cast<const char*>(frames[i].data()),frames[i].size()*sizeof(CAMERA_DEPTH_TYPE));
        require(bool(d),"write depth recording");
    }
}
struct Pipeline {
    CameraParams p;
    size_t pixels,voxels,slab;
    Device<CAMERA_DEPTH_TYPE> depth,input;
    Device<float3> vertices,normals,renderedVertices,renderedNormals,cameraPosition;
    Device<float> current,inverted;
    Device<uint> image,normalImage,depthImage;
    Device<VoxelType> guardedVolume;
    Device<VoxelWeightType> guardedWeights;
    CameraData data;
    PointInfo rendered;
    VolumeIntegrator integrator;
    Raycaster raycaster;
    ICP icp;
    explicit Pipeline(CameraParams params):p(params),pixels(p.m_width*p.m_height),voxels(size_t(VOLUME_SIZE)*VOLUME_SIZE*VOLUME_SIZE),slab(size_t(VOLUME_SIZE)*VOLUME_SIZE),depth(pixels),input(pixels),vertices(pixels),normals(pixels),renderedVertices(pixels),renderedNormals(pixels),cameraPosition(1),current(16),inverted(16),image(pixels),normalImage(pixels),depthImage(pixels),guardedVolume(voxels+2*slab),guardedWeights(voxels+2*slab),data{depth.p,vertices.p,normals.p},rendered{renderedVertices.p,renderedNormals.p},integrator(&p),raycaster(&p),icp(dim3(iDivUp(p.m_width,BLOCK_SIZE),iDivUp(p.m_height,BLOCK_SIZE)),blockSize,&p) {
        CameraHandler::SetParameters(p);
        std::vector<VoxelType> v(guardedVolume.n);for(auto& item:v)item.tsdf=123.25f;guardedVolume.put(v);
        guardedWeights.put(std::vector<VoxelWeightType>(guardedWeights.n,213));
        General::Instance()->InitVoxelArray(volume(),weights());sync();setPose(pose());
    }
    VoxelType* volume() { return guardedVolume.p+slab; }
    VoxelWeightType* weights() { return guardedWeights.p+slab; }
    void setPose(const Pose& t) { current.put(std::vector<float>(t.begin(),t.end()));General::Instance()->UpdateTransforms(current.p,inverted.p,cameraPosition.p,nullptr);sync(); }
    Pose getPose() { auto v=current.get();Pose t;std::copy(v.begin(),v.end(),t.begin());return t; }
    void convert() { CameraHandler::updateCameraData(dim3(iDivUp(p.m_width,BLOCK_SIZE),iDivUp(p.m_height,BLOCK_SIZE)),blockSize,data,input.p,current.p,normalImage.p,depthImage.p);sync(); }
    void upload(const std::vector<CAMERA_DEPTH_TYPE>& d) { input.put(d);convert(); }
    void integrate() { integrator.Integrate(volume(),weights(),inverted.p,data,cameraPosition.p);sync(); }
    void render() { check(cudaMemset(image.p,0,pixels*sizeof(uint)),"clear render image");raycaster.Raycast(image.p,current.p,volume(),rendered,cameraPosition.p,false);sync(); }
    void guardCheck() {
        auto v=guardedVolume.get();auto w=guardedWeights.get();
        for(size_t i=0;i<slab;++i) {
            require(v[i].tsdf==123.25f&&v[slab+voxels+i].tsdf==123.25f,"integration changed TSDF guard slab");
            require(w[i]==213&&w[slab+voxels+i]==213,"integration changed weight guard slab");
        }
    }
};
void testDepthAndReplay(const fs::path& out) {
    CameraParams p=parameters();auto depths=scene(p,pose(),true);depths[0]=0;depths[1]=2001;
    writeRecording(out/"replay-test",p,{depths});
    std::vector<CAMERA_RGB_TYPE> rgb(depths.size());
    for(size_t i=0;i<rgb.size();++i)rgb[i]=0xff000000u|uint(i);
    {
        std::ofstream f(out/"replay-test"/"recorded-rgb1",std::ios::binary);
        f.write(reinterpret_cast<const char*>(rgb.data()),rgb.size()*sizeof(CAMERA_RGB_TYPE));
        require(bool(f),"write RGB recording");
    }
    Pipeline gpu(p);
    std::string directory=(out/"replay-test").string();
    CameraFromFile replay(directory.data());require(replay.IsInitialized(),"recorded input initialization");
    require(replay.GetDepthFrame(gpu.input.p),"read first replay frame");
    require(gpu.input.get()==depths,"recorded uint16 depth byte roundtrip");
    require(replay.GetRgbFrame(gpu.image.p)&&gpu.image.get()==rgb,"recorded uint32 RGB byte roundtrip");
    require(!replay.GetDepthFrame(gpu.input.p),"end of replay must return false");
    gpu.convert();auto truth=referenceMap(p,depths,pose());auto v=gpu.vertices.get(),n=gpu.normals.get();
    float maxVertex=0,maxNormal=0;
    for(size_t i=0;i<v.size();++i) {
        require(valid(v[i])==valid(truth.vertices[i]),"depth validity mask differs from reference");
        if(valid(v[i]))maxVertex=std::max(maxVertex,norm(sub(v[i],truth.vertices[i])));
        maxNormal=std::max(maxNormal,norm(sub(n[i],truth.normals[i])));
    }
    require(maxVertex<.002f,"depth vertex reference error exceeds 0.002 mm");
    require(maxNormal<.0001f,"normal reference error exceeds 0.0001");
    std::cout<<"PASS replay_depth_normals: "<<v.size()<<" pixels; maximum vertex error "<<maxVertex<<" mm, normal error "<<maxNormal<<'\n';

    // A malformed frame must not silently reuse bytes from an earlier frame.
    for(int delta:{-1,1}) {
        fs::path dir=out/(delta<0?"short-frame":"long-frame");writeRecording(dir,p,{depths});
        fs::resize_file(dir/"recorded1",depths.size()*sizeof(CAMERA_DEPTH_TYPE)+delta);
        std::string path=dir.string();CameraFromFile bad(path.data());
        require(bad.IsInitialized()&&!bad.GetDepthFrame(gpu.input.p),"incorrect depth payload length was accepted");
    }
    for(const std::string& calibration:{std::string("0 73\n-100 -100 48 36\n0\n100\n2000\n"),
        std::string("97 73\n0 -100 48 36\n0\n100\n2000\n"),
        std::string("97 73\n-100 -100 48 36\n0\n2000\n100\n"),
        std::string("97 73\n-100 -100 48 36\n1\n100\n2000\n"),std::string("97 73\n-100")}) {
        fs::path dir=out/"bad-calibration";fs::create_directories(dir);
        {std::ofstream f(dir/"Parameters.txt");f<<calibration;}
        std::string path=dir.string();CameraFromFile bad(path.data());require(!bad.IsInitialized(),"invalid or unsupported calibration was accepted");
    }
    std::cout<<"PASS rejected_recordings: short/oversized depth, zero dimensions/focal length, reversed depth interval, nonzero unsupported z offset and incomplete calibration\n";
}
void testCorrespondence() {
    CameraParams p=parameters();Pipeline gpu(p);auto d=scene(p,pose());d[5]=0;d[66]=2001;gpu.upload(d);
    auto current=referenceMap(p,d,pose());
    for(auto& v:current.vertices)if(valid(v))v=add(v,vec(1.5f,-.75f,2.f));
    current.vertices[17]=vec(BAD_VERTEX,BAD_VERTEX,BAD_VERTEX);current.normals[87]=vec(0,0,0);
    gpu.renderedVertices.put(current.vertices);gpu.renderedNormals.put(current.normals);
    size_t blocks=size_t(iDivUp(p.m_width,BLOCK_SIZE))*iDivUp(p.m_height,BLOCK_SIZE);
    Device<float> aa(blocks*36),ab(blocks*6);
    check(cudaMemset(aa.p,0,aa.n*sizeof(float)),"clear AtA");check(cudaMemset(ab.p,0,ab.n*sizeof(float)),"clear Atb");
    computeCorrespondenceForTest(gpu.data,gpu.rendered,aa.p,ab.p,gpu.inverted.p,p);sync();
    auto actualAA=aa.get(),actualAB=ab.get();std::vector<double> expectedAA(aa.n),expectedAB(ab.n);
    auto newV=gpu.vertices.get(),newN=gpu.normals.get();Pose inv=inverse(pose());size_t accepted=0;
    for(unsigned y=0;y<p.m_height;++y)for(unsigned x=0;x<p.m_width;++x) {
        size_t i=y*p.m_width+x;if(d[i]<p.m_min_depth||d[i]>p.m_max_depth||!valid(current.vertices[i]))continue;
        float3 c=transform(inv,current.vertices[i]);if(std::abs(c.z)<_EPSILON_)continue;
        int px=int(std::floor((p.m_intrinsic.m_fx*c.x+p.m_intrinsic.m_cx*c.z)/c.z+.5f));
        int py=int(std::floor((p.m_intrinsic.m_fy*c.y+p.m_intrinsic.m_cy*c.z)/c.z+.5f));
        if(px<0||px>=int(p.m_width)||py<0||py>=int(p.m_height)||c.z>=-_EPSILON_)continue;
        size_t j=py*p.m_width+px;float3 nv=newV[j],cn=current.normals[i];float ndot=dot3(newN[j],cn);
        if(!valid(nv)||norm(sub(nv,current.vertices[i]))>=DISTANCE_THRESHOLD||ndot<=NORMAL_THRESHOLD||ndot>=1+_EPSILON_)continue;
        float3 r=cross3(nv,cn);double row[]={-r.x,-r.y,-r.z,cn.x,cn.y,cn.z};double b=dot3(cn,sub(current.vertices[i],nv));
        size_t block=(y/BLOCK_SIZE)*iDivUp(p.m_width,BLOCK_SIZE)+x/BLOCK_SIZE;
        for(int a=0;a<6;++a){expectedAB[block*6+a]+=row[a]*b;for(int z=a;z<6;++z)expectedAA[block*36+a*6+z]+=row[a]*row[z];}++accepted;
    }
    double maxScaled=0;
    for(size_t b=0;b<blocks;++b)for(int a=0;a<6;++a){
        size_t i=b*6+a;double tolerance=.02+std::abs(expectedAB[i])*2e-5;
        maxScaled=std::max(maxScaled,std::abs(actualAB[i]-expectedAB[i])/tolerance);
        require(std::isfinite(actualAB[i])&&std::abs(actualAB[i]-expectedAB[i])<=tolerance,"ICP Atb differs from CPU reference");
        for(int z=a;z<6;++z){size_t j=b*36+a*6+z;double t=.1+std::abs(expectedAA[j])*2e-5;maxScaled=std::max(maxScaled,std::abs(actualAA[j]-expectedAA[j])/t);require(std::isfinite(actualAA[j])&&std::abs(actualAA[j]-expectedAA[j])<=t,"ICP AtA differs from CPU reference");}
    }
    // Check all upper-triangle numerical values before the added symmetric-output contract.
    for(size_t b=0;b<blocks;++b)for(int a=0;a<6;++a)for(int z=a;z<6;++z)
        require(actualAA[b*36+a*6+z]==actualAA[b*36+z*6+a],"ICP AtA symmetric entry was not initialized");
    require(accepted>1000,"too few correspondence fixture matches");
    std::cout<<"PASS correspondence_cpu_reference: "<<blocks<<" blocks, "<<accepted<<" matches, maximum normalized tolerance "<<maxScaled<<"; includes partial blocks and invalid pixels\n";
}
void testIntegrationAndRaycast() {
    CameraParams p=parameters();
    // This field of view sees the final x slab near z=0. The original x+1
    // storage error therefore overwrites a suffix canary, as well as shifting
    // the integrated volume relative to the independent CPU reference.
    p.m_intrinsic.m_fx=p.m_intrinsic.m_fy=-float(p.m_width)*.6f;
    p.m_invIntrinsic.Set(1/p.m_intrinsic.m_fx,1/p.m_intrinsic.m_fy,-p.m_intrinsic.m_cx/p.m_intrinsic.m_fx,-p.m_intrinsic.m_cy/p.m_intrinsic.m_fy);
    Pipeline gpu(p);auto d=scene(p,pose(),true);gpu.upload(d);gpu.integrate();gpu.guardCheck();
    auto v=gpu.guardedVolume.get();auto w=gpu.guardedWeights.get();auto input=referenceMap(p,d,pose());
    Pose inv=inverse(pose());float ratio=float(worldVolume)/VOLUME_SIZE;float maximum=0;size_t updated=0,boundaryVoxels=0,boundaryDecisions=0;
    for(unsigned x=0;x<VOLUME_SIZE;++x)for(unsigned y=0;y<VOLUME_SIZE;++y)for(unsigned z=0;z<VOLUME_SIZE;++z) {
        size_t i=gpu.slab+(size_t(x)*VOLUME_SIZE+y)*VOLUME_SIZE+z;float expected=TRUNCATION;int weight=0;float sdf=std::numeric_limits<float>::quiet_NaN();
        float3 world=vec(x*ratio-worldVolume*.5f+ratio*.5f,y*ratio-worldVolume*.5f+ratio*.5f,z*ratio-worldVolume*.5f+ratio*.5f);
        float3 camera=transform(inv,world);
        int px=int(std::floor((p.m_intrinsic.m_fx*camera.x+p.m_intrinsic.m_cx*camera.z)/camera.z+.5f));
        int py=int(std::floor((p.m_intrinsic.m_fy*camera.y+p.m_intrinsic.m_cy*camera.z)/camera.z+.5f));
        if(px>=2&&px<int(p.m_width)-2&&py>=2&&py<int(p.m_height)-2&&std::abs(camera.z)<=p.m_max_depth&&std::abs(camera.z)>=p.m_min_depth) {
            float3 surface=input.vertices[py*p.m_width+px];
            if(valid(surface)){sdf=norm(sub(surface,vec(0,0,CAMERA_START_Z_MM)))-norm(sub(world,vec(0,0,CAMERA_START_Z_MM)));if(sdf>=-TRUNCATION){expected=std::min(sdf,float(TRUNCATION));weight=1;}}
        }
        // CPU and GPU evaluation can straddle the hard -TRUNCATION threshold
        // by a few floating-point ULPs. Only within this explicit 0.002 mm
        // band, accept either correctly unobserved or correctly updated state.
        if(std::isfinite(sdf)&&std::abs(sdf+TRUNCATION)<=.002f) {
            ++boundaryVoxels;if(w[i]!=weight)++boundaryDecisions;
            require(w[i]<=1,"unexpected TSDF weight in truncation boundary band");
            weight=w[i];expected=weight?std::min(sdf,float(TRUNCATION)):float(TRUNCATION);
        }
        if(w[i]!=weight) {
            std::ostringstream message;message<<std::setprecision(10)<<"TSDF integration weight differs from CPU reference at voxel ("<<x<<','<<y<<','<<z<<"), projected pixel ("<<px<<','<<py<<"), CPU weight "<<weight<<", GPU weight "<<int(w[i])<<", CPU TSDF "<<expected<<", GPU TSDF "<<v[i].tsdf<<", camera z "<<camera.z;
            throw std::runtime_error(message.str());
        }
        updated+=weight;
        maximum=std::max(maximum,std::abs(v[i].tsdf-expected));
        require(std::isfinite(v[i].tsdf)&&std::abs(v[i].tsdf-expected)<.002f,"TSDF integration differs from CPU reference by >= 0.002 mm");
    }
    require(updated>10000,"TSDF fixture integrated too few voxels");gpu.render();auto rv=gpu.renderedVertices.get(),rn=gpu.renderedNormals.get();
    size_t hits=0,unitNormals=0,zeroNormals=0,facingNormals=0,observedInterior=0,observedFacing=0;double squared=0,observedAngleSum=0;float maxSurface=0,maxNormalError=0;
    for(size_t i=0;i<rv.size();++i)if(valid(rv[i])) {
        ++hits;squared+=double(rv[i].z)*rv[i].z;maxSurface=std::max(maxSurface,std::abs(rv[i].z));
        float length=norm(rn[i]);require(std::isfinite(length),"nonfinite raycast normal");
        if(std::abs(length-1)<.001f)++unitNormals;if(length<.001f)++zeroNormals;if(rn[i].z>0)++facingNormals;
        int x=int(std::floor((rv[i].x/worldVolume+.5f)*VOLUME_SIZE));
        int y=int(std::floor((rv[i].y/worldVolume+.5f)*VOLUME_SIZE));
        int z=int(std::floor((rv[i].z/worldVolume+.5f)*VOLUME_SIZE));
        if(x<=0||x+1>=int(VOLUME_SIZE)||y<=0||y+1>=int(VOLUME_SIZE)||z<=0||z+1>=int(VOLUME_SIZE))continue;
        auto index=[&](int a,int b,int c){return gpu.slab+(size_t(a)*VOLUME_SIZE+b)*VOLUME_SIZE+c;};
        float3 expectedNormal=unit(vec(v[index(x+1,y,z)].tsdf-v[index(x-1,y,z)].tsdf,v[index(x,y+1,z)].tsdf-v[index(x,y-1,z)].tsdf,v[index(x,y,z+1)].tsdf-v[index(x,y,z-1)].tsdf));
        maxNormalError=std::max(maxNormalError,norm(sub(rn[i],expectedNormal)));
        if(w[index(x+1,y,z)]&&w[index(x-1,y,z)]&&w[index(x,y+1,z)]&&w[index(x,y-1,z)]&&w[index(x,y,z+1)]&&w[index(x,y,z-1)]) {
            ++observedInterior;if(rn[i].z>0&&std::abs(length-1)<.001f)++observedFacing;
            observedAngleSum+=std::acos(std::clamp(double(rn[i].z)/(length>0?length:1),-1.0,1.0))*180/MATH_PI;
        }
    }
    double rmse=hits?std::sqrt(squared/hits):1e9;
    require(hits>gpu.pixels/2,"raycast plane coverage below 50 percent");require(rmse<ratio*1.5+1.5,"raycast plane error exceeds voxel and step allowance");
    std::cout<<"plane normal diagnostics: "<<hits<<" hits, "<<unitNormals<<" unit, "<<zeroNormals<<" zero, "<<facingNormals<<" facing +z; "<<observedInterior<<" with six observed interior neighbors, "<<observedFacing<<" of those unit and facing +z, mean analytic-plane angle "<<(observedInterior?observedAngleSum/observedInterior:0)<<" degrees; maximum CPU normal-reference error "<<maxNormalError<<'\n';
    require(maxNormalError<.0001f,"raycast normal differs from CPU central-difference reference");
    require(unitNormals>hits/2,"fewer than half of plane ray hits have unit-length normals");
    require(unitNormals+zeroNormals==hits,"raycast normals are neither unit nor zero vectors");
    gpu.guardCheck();
    std::cout<<"PASS integration_cpu_reference_and_raycast: "<<updated<<" updated voxels, max TSDF error "<<maximum<<" mm, "<<boundaryVoxels<<" voxels in the 0.002 mm truncation-boundary band ("<<boundaryDecisions<<" CPU/GPU weight decisions differ), "<<hits<<" ray hits, plane RMSE "<<rmse<<" mm, maximum "<<maxSurface<<" mm; guard slabs unchanged\n";
}
void testICP() {
    CameraParams p=parameters();Pipeline gpu(p);Pose start=pose();auto previous=referenceMap(p,scene(p,start),start);
    gpu.renderedVertices.put(previous.vertices);gpu.renderedNormals.put(previous.normals);
    Pose truth=pose(5,-3,CAMERA_START_Z_MM+2,.3f*MATH_PI/180);
    gpu.upload(scene(p,truth));gpu.icp.Iterate_ICP(gpu.data,gpu.rendered,gpu.current.p);sync();Pose actual=gpu.getPose();
    float te=translationError(actual,truth),re=rotationError(actual,truth);
    require(gpu.icp.LastSolveSucceeded(),"known-motion ICP did not solve all iterations");
    require(te<3.f&&re<.3f,"known-motion ICP exceeds 3 mm / 0.3 degree tolerance");
    require(te<translationError(start,truth),"known-motion ICP did not improve initial translation");
    require(re<rotationError(start,truth)*.9f,"known-motion ICP did not improve initial rotation by at least 10 percent");
    std::cout<<"PASS known_motion_icp: "<<gpu.icp.LastSolveIterations()<<" solves, translation error "<<te<<" mm, rotation error "<<re<<" degrees\n";
    gpu.setPose(start);auto flat=scene(p,start,true);gpu.upload(flat);auto plane=referenceMap(p,flat,start);
    gpu.renderedVertices.put(plane.vertices);gpu.renderedNormals.put(plane.normals);
    gpu.icp.Iterate_ICP(gpu.data,gpu.rendered,gpu.current.p);sync();
    require(!gpu.icp.LastSolveSucceeded(),"rank-deficient plane ICP must report rejection");
    require(translationError(gpu.getPose(),start)<1e-5&&rotationError(gpu.getPose(),start)<.01,"rejected ICP changed pose");
    std::cout<<"PASS degenerate_icp_rejection: plane has unconstrained motion; input pose retained\n";
}
void testBehindCamera() {
    CameraParams p=parameters(13,11);Pipeline gpu(p);gpu.setPose(pose(0,0,0));
    gpu.upload(std::vector<CAMERA_DEPTH_TYPE>(gpu.pixels,300));gpu.integrate();gpu.guardCheck();
    auto weights=gpu.guardedWeights.get();size_t frontObserved=0;
    for(unsigned x=0;x<VOLUME_SIZE;++x)for(unsigned y=0;y<VOLUME_SIZE;++y)for(unsigned z=0;z<VOLUME_SIZE;++z) {
        size_t index=gpu.slab+(size_t(x)*VOLUME_SIZE+y)*VOLUME_SIZE+z;
        if(z>=VOLUME_SIZE/2)require(weights[index]==0,"volume integration updated a voxel behind the camera");
        else if(weights[index])++frontObserved;
    }
    require(frontObserved>100,"inside-volume camera fixture integrated too few front-facing voxels");
    std::cout<<"PASS behind_camera_rejection: camera inside volume, "<<frontObserved<<" front-facing observed voxels, no behind-camera integration\n";
}
void ppm(const fs::path& path,unsigned width,unsigned height,const std::vector<uint>& pixels) {
    std::ofstream f(path,std::ios::binary);f<<"P6\n"<<width<<' '<<height<<"\n255\n";
    for(uint pixel:pixels){char rgb[]={char(pixel&255),char((pixel>>8)&255),char((pixel>>16)&255)};f.write(rgb,3);}require(bool(f),"write PPM");
}
void ply(const fs::path& path,const std::vector<float3>& vertices,const std::vector<float3>& normals) {
    size_t count=0;for(auto v:vertices)if(valid(v))++count;
    std::ofstream f(path);f<<"ply\nformat ascii 1.0\ncomment Synthetic fixture, raycast samples in millimeters; not a triangle mesh\nelement vertex "<<count<<"\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n";
    for(size_t i=0;i<vertices.size();++i)if(valid(vertices[i])){auto v=vertices[i],n=normals[i];f<<v.x<<' '<<v.y<<' '<<v.z<<' '<<n.x<<' '<<n.y<<' '<<n.z<<'\n';}require(bool(f),"write PLY");
}
void demo(const fs::path& out) {
    fs::create_directories(out);CameraParams p=parameters(193,145);constexpr int frames=12;
    std::vector<Pose> truth;std::vector<std::vector<CAMERA_DEPTH_TYPE>> recordings;
    for(int i=0;i<frames;++i){truth.push_back(pose(float(i)*.7f,float(i)*-.3f,CAMERA_START_Z_MM+float(i)*.2f,float(i)*.025f*MATH_PI/180));recordings.push_back(scene(p,truth.back()));}
    writeRecording(out/"recording",p,recordings);std::string directory=(out/"recording").string();CameraFromFile replay(directory.data());require(replay.IsInitialized(),"demo replay initialization");Pipeline gpu(p);
    std::ofstream metrics(out/"metrics.json");metrics<<std::setprecision(9)<<"{\n  \"input\": \"synthetic analytic three-sphere scene and back plane, uint16 axial depth in mm\",\n  \"operators\": \"CameraFromFile -> CUDA vertices/normals -> ICP -> TSDF integration -> CUDA raycast\",\n  \"volume_size\": "<<VOLUME_SIZE<<",\n  \"volume_width_mm\": "<<worldVolume<<",\n  \"voxel_width_mm\": "<<float(worldVolume)/VOLUME_SIZE<<",\n  \"image_width\": "<<p.m_width<<",\n  \"image_height\": "<<p.m_height<<",\n  \"frames\": [\n";
    float maximumTranslation=0,maximumRotation=0;size_t finalHits=0;double finalRmse=0,finalRayRmse=0;Pose finalEstimate=pose();
    for(int i=0;i<frames;++i) {
        require(replay.GetDepthFrame(gpu.input.p),"demo missing recorded frame");gpu.convert();
        if(i){gpu.icp.Iterate_ICP(gpu.data,gpu.rendered,gpu.current.p);sync();require(gpu.icp.LastSolveSucceeded(),"demo ICP rejected frame "+std::to_string(i));General::Instance()->UpdateTransforms(gpu.current.p,gpu.inverted.p,gpu.cameraPosition.p,nullptr);sync();}
        gpu.integrate();gpu.render();auto estimate=gpu.getPose();float te=translationError(estimate,truth[i]),re=rotationError(estimate,truth[i]);maximumTranslation=std::max(te,maximumTranslation);maximumRotation=std::max(re,maximumRotation);
        auto rv=gpu.renderedVertices.get();size_t hits=0,outliers=0;double surfaceSum=0,raySum=0;std::vector<float> surfaceErrors,rayErrors;std::array<size_t,4> surfaceCounts{};
        const float voxelWidth=float(worldVolume)/VOLUME_SIZE;
        const float rayOutlierThreshold=2*voxelWidth+1.5f;
        for(unsigned y=0;y<p.m_height;++y)for(unsigned x=0;x<p.m_width;++x){size_t j=y*p.m_width+x;if(!valid(rv[j]))continue;float z=sceneDepth(p,x,y,estimate);float3 expected=transform(estimate,unproject(p,x,y,z));float surfaceError=sceneSurfaceDistance(rv[j]),rayError=norm(sub(rv[j],expected));surfaceSum+=double(surfaceError)*surfaceError;raySum+=double(rayError)*rayError;surfaceErrors.push_back(surfaceError);rayErrors.push_back(rayError);if(rayError>rayOutlierThreshold)++outliers;size_t surface=0;float closest=std::abs(rv[j].z+300.f);for(size_t s=0;s<spheres.size();++s){float error=std::abs(norm(sub(rv[j],spheres[s].center))-spheres[s].radius);if(error<closest){closest=error;surface=s+1;}}if(closest<voxelWidth*2+1.5f)++surfaceCounts[surface];++hits;}
        double rmse=hits?std::sqrt(surfaceSum/hits):1e9,rayRmse=hits?std::sqrt(raySum/hits):1e9;float surfaceP95=percentile(surfaceErrors,.95f),rayP95=percentile(rayErrors,.95f);finalHits=hits;finalRmse=rmse;finalRayRmse=rayRmse;finalEstimate=estimate;
        require(hits>gpu.pixels/2,"demo raycast coverage below 50 percent");require(te<12&&re<1.5,"demo estimated pose exceeds 12 mm / 1.5 degree allowance");
        // World-space surface error and per-ray visibility error measure
        // different things. At a silhouette, voxel occupancy can move a ray
        // between a foreground sphere and the distant back plane. Keep both.
        require(rmse<voxelWidth*1.5f+1.5f&&surfaceP95<voxelWidth*2+1.5f,"demo point-to-analytic-surface geometry exceeds voxel-scale allowance");
        require(double(outliers)/hits<.05,"more than 5 percent of demo ray hits exceed the two-voxel plus ray-step visibility allowance");
        for(size_t s=1;s<surfaceCounts.size();++s)require(surfaceCounts[s]>=100,"demo did not reconstruct at least 100 nearby raycast samples on sphere "+std::to_string(s));
        std::ostringstream filename;filename<<"reconstruction-"<<std::setw(2)<<std::setfill('0')<<i<<".ppm";ppm(out/filename.str(),p.m_width,p.m_height,gpu.image.get());
        if(i)metrics<<",\n";metrics<<"    {\"index\": "<<i<<", \"icp_solved\": "<<(i?(gpu.icp.LastSolveSucceeded()?"true":"false"):"null")<<", \"icp_iterations\": "<<(i?gpu.icp.LastSolveIterations():0)<<", \"translation_error_mm\": "<<te<<", \"rotation_error_degrees\": "<<re<<", \"raycast_hits\": "<<hits<<", \"analytic_surface_hit_counts\": {\"back_plane\": "<<surfaceCounts[0]<<", \"sphere_1\": "<<surfaceCounts[1]<<", \"sphere_2\": "<<surfaceCounts[2]<<", \"sphere_3\": "<<surfaceCounts[3]<<"}, \"raycast_surface_rmse_mm\": "<<rmse<<", \"raycast_surface_p95_mm\": "<<surfaceP95<<", \"raycast_surface_max_mm\": "<<*std::max_element(surfaceErrors.begin(),surfaceErrors.end())<<", \"raycast_ray_distance_rmse_mm\": "<<rayRmse<<", \"raycast_ray_distance_p95_mm\": "<<rayP95<<", \"raycast_ray_distance_outlier_threshold_mm\": "<<rayOutlierThreshold<<", \"raycast_ray_distance_outlier_fraction\": "<<double(outliers)/hits<<", \"estimated_pose_row_major\": [";for(int k=0;k<16;++k){if(k)metrics<<", ";metrics<<estimate[k];}metrics<<"], \"ground_truth_pose_row_major\": [";for(int k=0;k<16;++k){if(k)metrics<<", ";metrics<<truth[i][k];}metrics<<"]}";
        std::cout<<"demo frame "<<i<<": "<<hits<<" ray hits, pose error "<<te<<" mm / "<<re<<" degrees, surface RMSE "<<rmse<<" mm / p95 "<<surfaceP95<<" mm, per-ray distance RMSE "<<rayRmse<<" mm / p95 "<<rayP95<<" mm, visibility outliers "<<100.*outliers/hits<<" percent, sphere samples "<<surfaceCounts[1]<<'/'<<surfaceCounts[2]<<'/'<<surfaceCounts[3]<<'\n';
    }
    gpu.guardCheck();
    const float initialTranslationError=translationError(pose(),truth.back());
    const float finalTranslationError=translationError(finalEstimate,truth.back());
    const float estimatedTravel=translationError(finalEstimate,pose());
    require(finalTranslationError<initialTranslationError*.75f,"demo tracking did not reduce final translation error by at least 25 percent compared with a stationary pose");
    require(estimatedTravel>initialTranslationError*.5f,"demo estimated camera traveled less than half the ground-truth displacement");
    ppm(out/"reconstruction.ppm",p.m_width,p.m_height,gpu.image.get());ppm(out/"input-depth.ppm",p.m_width,p.m_height,gpu.depthImage.get());ppm(out/"input-normals.ppm",p.m_width,p.m_height,gpu.normalImage.get());ply(out/"surface.ply",gpu.renderedVertices.get(),gpu.renderedNormals.get());
    auto weights=gpu.guardedWeights.get();size_t observed=0;int maxWeight=0;for(size_t j=gpu.slab;j<gpu.slab+gpu.voxels;++j){if(weights[j])++observed;maxWeight=std::max(maxWeight,int(weights[j]));}
    require(observed>10000&&maxWeight==frames,"demo volume integration weight evidence missing");
    metrics<<"\n  ],\n  \"maximum_translation_error_mm\": "<<maximumTranslation<<",\n  \"maximum_rotation_error_degrees\": "<<maximumRotation<<",\n  \"final_raycast_surface_rmse_mm\": "<<finalRmse<<",\n  \"final_raycast_ray_distance_rmse_mm\": "<<finalRayRmse<<",\n  \"stationary_pose_final_translation_error_mm\": "<<initialTranslationError<<",\n  \"estimated_camera_displacement_mm\": "<<estimatedTravel<<",\n  \"final_raycast_hits\": "<<finalHits<<",\n  \"observed_voxels\": "<<observed<<",\n  \"maximum_voxel_weight\": "<<maxWeight<<",\n  \"guard_slabs_unchanged\": true,\n  \"status\": \"passed\"\n}\n";require(bool(metrics),"write metrics");
    std::cout<<"PASS moving_replay_demo: "<<frames<<" frames, "<<observed<<" observed voxels; outputs "<<out<<'\n';
}
void replayRecording(const fs::path& directory,int frames,const fs::path& out) {
    require(frames>0,"--recording requires --frames with a positive frame count");
    fs::create_directories(out);std::string path=directory.string();CameraFromFile replay(path.data());
    require(replay.IsInitialized(),"recording calibration could not be initialized");
    CameraParams p=*replay.GetParameters();require(p.m_width>4&&p.m_height>4,"recording dimensions must exceed the integration border of four pixels");
    Pipeline gpu(p);std::ofstream metrics(out/"metrics.json");
    metrics<<std::setprecision(9)<<"{\n  \"input\": \"user-supplied Parameters.txt and recordedN depth frames\",\n  \"ground_truth_available\": false,\n  \"validation_scope\": \"operator execution and finite output; no reconstruction or pose accuracy measurement without ground truth\",\n  \"operators\": \"CameraFromFile -> CUDA vertices/normals -> ICP -> TSDF integration -> CUDA raycast\",\n  \"volume_size\": "<<VOLUME_SIZE<<",\n  \"volume_width_mm\": "<<worldVolume<<",\n  \"voxel_width_mm\": "<<float(worldVolume)/VOLUME_SIZE<<",\n  \"image_width\": "<<p.m_width<<",\n  \"image_height\": "<<p.m_height<<",\n  \"initial_camera_z_mm\": "<<CAMERA_START_Z_MM<<",\n  \"frames\": [\n";
    size_t finalHits=0;
    for(int i=0;i<frames;++i) {
        require(replay.GetDepthFrame(gpu.input.p),"recording frame "+std::to_string(i+1)+" is missing or has an incorrect byte length");gpu.convert();
        if(i){gpu.icp.Iterate_ICP(gpu.data,gpu.rendered,gpu.current.p);sync();require(gpu.icp.LastSolveSucceeded(),"recording ICP rejected frame "+std::to_string(i+1)+"; stopped before integrating an untracked frame");General::Instance()->UpdateTransforms(gpu.current.p,gpu.inverted.p,gpu.cameraPosition.p,nullptr);sync();}
        gpu.integrate();gpu.render();auto estimated=gpu.getPose();for(float element:estimated)require(std::isfinite(element),"recording pose contains nonfinite values");
        auto vertices=gpu.renderedVertices.get(),normals=gpu.renderedNormals.get();size_t hits=0;
        for(size_t j=0;j<vertices.size();++j)if(valid(vertices[j])){++hits;require(std::isfinite(norm(normals[j])),"recording raycast normal is nonfinite");}
        require(hits>0,"recording produced no raycast surface inside the fixed 1024 mm reconstruction cube");finalHits=hits;
        std::ostringstream filename;filename<<"reconstruction-"<<std::setw(4)<<std::setfill('0')<<i<<".ppm";ppm(out/filename.str(),p.m_width,p.m_height,gpu.image.get());
        if(i)metrics<<",\n";metrics<<"    {\"index\": "<<i<<", \"icp_solved\": "<<(i?"true":"null")<<", \"icp_iterations\": "<<(i?gpu.icp.LastSolveIterations():0)<<", \"raycast_hits\": "<<hits<<", \"estimated_pose_row_major\": [";for(int k=0;k<16;++k){if(k)metrics<<", ";metrics<<estimated[k];}metrics<<"]}";
        std::cout<<"recording frame "<<i+1<<": "<<hits<<" ray hits; "<<(i?"ICP solved":"initialized pose")<<'\n';
    }
    gpu.guardCheck();auto weights=gpu.guardedWeights.get();size_t observed=0;int maxWeight=0;for(size_t i=gpu.slab;i<gpu.slab+gpu.voxels;++i){if(weights[i])++observed;maxWeight=std::max(maxWeight,int(weights[i]));}
    ppm(out/"reconstruction.ppm",p.m_width,p.m_height,gpu.image.get());ppm(out/"input-depth.ppm",p.m_width,p.m_height,gpu.depthImage.get());ppm(out/"input-normals.ppm",p.m_width,p.m_height,gpu.normalImage.get());ply(out/"surface.ply",gpu.renderedVertices.get(),gpu.renderedNormals.get());
    metrics<<"\n  ],\n  \"final_raycast_hits\": "<<finalHits<<",\n  \"observed_voxels\": "<<observed<<",\n  \"maximum_voxel_weight\": "<<maxWeight<<",\n  \"guard_slabs_unchanged\": true,\n  \"status\": \"executed\"\n}\n";require(bool(metrics),"write recording metrics");
    std::cout<<"PASS supplied_recording_execution: "<<frames<<" frames, "<<observed<<" observed voxels; no ground-truth accuracy claim\n";
}
}
int main(int argc,char** argv) {
    try {
        bool tests=false,runDemo=false;fs::path out="openfusion-output",recording;int recordingFrames=0;
        for(int i=1;i<argc;++i){std::string arg=argv[i];if(arg=="--self-test")tests=true;else if(arg=="--demo")runDemo=true;else if(arg=="--output"&&i+1<argc)out=argv[++i];else if(arg=="--recording"&&i+1<argc)recording=argv[++i];else if(arg=="--frames"&&i+1<argc){std::string value=argv[++i];size_t end=0;recordingFrames=std::stoi(value,&end);require(end==value.size()&&recordingFrames>0,"--frames must be a positive integer");}else throw std::runtime_error("usage: openfusion_headless [--self-test] [--demo] [--recording DIRECTORY --frames N] [--output DIRECTORY]");}
        require(recordingFrames==0||!recording.empty(),"--frames requires --recording");
        if(!tests&&!runDemo&&recording.empty())tests=runDemo=true;
        int device=0;check(cudaGetDevice(&device),"get CUDA device");cudaDeviceProp properties{};check(cudaGetDeviceProperties(&properties,device),"get CUDA properties");
        std::cout<<"CUDA device: "<<properties.name<<"; compute capability "<<properties.major<<'.'<<properties.minor<<"; volume "<<VOLUME_SIZE<<"^3, width "<<worldVolume<<" mm\n";
        fs::create_directories(out);
        if(tests){testDepthAndReplay(out);testCorrespondence();testIntegrationAndRaycast();testICP();testBehindCamera();}
        if(runDemo)demo(out/"demo");if(!recording.empty())replayRecording(recording,recordingFrames,out/"replay");sync();std::cout<<"All requested checks passed.\n";return 0;
    }catch(const std::exception& error){std::cerr<<"FAIL: "<<error.what()<<'\n';return 1;}
}

(function(){
  'use strict';
  const mul=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0)));
  const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
  function tilt(t,pitch){
    const original=Math.atan2(-t[2][2],Math.hypot(t[0][2],t[1][2]))*180/Math.PI;
    const q=-(pitch-original)*Math.PI/180,c=Math.cos(q),s=Math.sin(q);
    return mul(t,[[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]]);
  }
  function planes(t,k,zmin,zmax,baseline=0){
    const xmin=-k.ppx/k.fx,xmax=(k.width-k.ppx)/k.fx;
    const ymin=-k.ppy/k.fy,ymax=(k.height-k.ppy)/k.fy;
    const local=[[1,0,-xmin,-baseline],[-1,0,xmax,0],[0,1,-ymin,0],[0,-1,ymax,0],[0,0,1,-zmin],[0,0,-1,zmax]];
    const center=t.slice(0,3).map(r=>r[3]);
    return local.map(p=>{const n=t.slice(0,3).map(row=>dot(row.slice(0,3),p.slice(0,3)));return [...n,p[3]-dot(n,center)];});
  }
  function clip(polygon,p){
    const value=q=>p[0]*q[0]+p[1]*q[1]+p[2],out=[];
    for(let i=0;i<polygon.length;i++){
      const a=polygon[i],b=polygon[(i+1)%polygon.length],va=value(a),vb=value(b);
      if(va>=-1e-10)out.push(a);
      if((va>=0)!==(vb>=0)){const s=va/(va-vb);out.push(a.map((x,j)=>x+s*(b[j]-x)));}
    }
    return out;
  }
  function footprint(all,z){
    let polygon=[[-10,-10],[10,-10],[10,10],[-10,10]];
    for(const p of all)polygon=clip(polygon,[p[0],p[1],p[2]*z+p[3]]);
    return polygon.map(p=>[...p,z]);
  }
  function centerRange(polygon){
    const xs=[];
    for(let i=0;i<polygon.length;i++){
      const a=polygon[i],b=polygon[(i+1)%polygon.length];
      if(Math.abs(a[1])<1e-9)xs.push(a[0]);
      if(a[1]*b[1]<0){const s=-a[1]/(b[1]-a[1]);xs.push(a[0]+s*(b[0]-a[0]));}
    }
    return xs.length?[Math.min(...xs),Math.max(...xs)]:null;
  }
  function fov(k){return [Math.atan(k.ppx/k.fx)+Math.atan((k.width-k.ppx)/k.fx),Math.atan(k.ppy/k.fy)+Math.atan((k.height-k.ppy)/k.fy)].map(x=>x*180/Math.PI);}
  function evaluate(data,modeIndex,pitch,height){
    const mode=data.modes[modeIndex],camera=tilt(data.camera,pitch),depth=mul(camera,data.color_from_depth);
    const rgbPlanes=planes(camera,mode.rgb,.001,data.display_depth_m);
    const depthPlanes=planes(depth,mode.depth,mode.min_z_m,data.display_depth_m,data.baseline_m);
    const z=data.ground+height,all=[...rgbPlanes,...depthPlanes];
    return {camera,depth,rgb:footprint(rgbPlanes,z),depthArea:footprint(depthPlanes,z),both:footprint(all,z),
      range:centerRange(footprint(all,z)),groundRange:centerRange(footprint(all,data.ground)),rgbFov:fov(mode.rgb)};
  }
  const api={mul,tilt,planes,clip,footprint,centerRange,fov,evaluate};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;else globalThis.CameraViewGeometry=api;
})();

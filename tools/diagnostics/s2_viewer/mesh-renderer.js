(function(){
'use strict';
function create(canvas,data,math){
  const gl=data.display.software_rendering?null:canvas.getContext('webgl',{alpha:true,antialias:true,premultipliedAlpha:false});
  if(!gl)return createSoftware(canvas,data,math);
  const {mul,point}=math;
  const I=()=>[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
  const flatten=m=>new Float32Array([0,1,2,3].flatMap(j=>m.map(r=>r[j])));
  function shader(type,source){const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;}
  const program=gl.createProgram();
  gl.attachShader(program,shader(gl.VERTEX_SHADER,`attribute vec3 a_position;attribute vec3 a_normal;uniform mat4 u_model;uniform mat4 u_projection;varying vec3 v_normal;void main(){gl_Position=u_projection*u_model*vec4(a_position,1.0);v_normal=mat3(u_model)*a_normal;}`));
  gl.attachShader(program,shader(gl.FRAGMENT_SHADER,`precision mediump float;varying vec3 v_normal;uniform vec3 u_color;uniform float u_ambient;void main(){vec3 n=normalize(v_normal);if(!gl_FrontFacing)n=-n;float light=u_ambient+(1.0-u_ambient)*(.77*max(0.0,dot(n,normalize(vec3(.4,-.6,1.0))))+.23*max(0.0,dot(n,normalize(vec3(-.5,.5,.3)))));gl_FragColor=vec4(u_color*light,1.0);}`));
  gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);
  const position=gl.getAttribLocation(program,'a_position'),normal=gl.getAttribLocation(program,'a_normal'),model=gl.getUniformLocation(program,'u_model'),projection=gl.getUniformLocation(program,'u_projection'),color=gl.getUniformLocation(program,'u_color');
  const ambient=gl.getUniformLocation(program,'u_ambient');
  gl.enableVertexAttribArray(position);gl.enableVertexAttribArray(normal);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);
  function makeBuffer(vertices,faces,mode){
    const expanded=[];
    if(mode===gl.LINES){for(const p of vertices)expanded.push(...p,0,0,1);}
    else for(let i=0;i<faces.length;i+=3){const a=vertices[faces[i]],b=vertices[faces[i+1]],c=vertices[faces[i+2]],u=b.map((v,k)=>v-a[k]),v=c.map((v,k)=>v-a[k]),n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],length=Math.hypot(...n);if(length<1e-14)continue;for(const p of [a,b,c])expanded.push(...p,...n.map(v=>v/length));}
    const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(expanded),gl.STATIC_DRAW);
    return {buffer,count:expanded.length/6,mode:mode||gl.TRIANGLES};
  }
  function uint16(b64){const raw=atob(b64),bytes=new Uint8Array(raw.length);for(let i=0;i<raw.length;i++)bytes[i]=raw.charCodeAt(i);return new Uint16Array(bytes.buffer);}
  const meshes={};
  for(const [name,m]of Object.entries(data.surfaces.meshes)){
    const q=uint16(m.vertices),v=[];for(let i=0;i<q.length;i+=3)v.push([0,1,2].map(k=>m.min[k]+q[i+k]/65535*m.span[k]));
    meshes[name]=makeBuffer(v,uint16(m.faces));
  }
  function box(size,center=[0,0,0]){const v=[];for(const z of [-1,1])for(const y of [-1,1])for(const x of [-1,1])v.push([x,y,z].map((n,i)=>center[i]+n*size[i]/2));return makeBuffer(v,[0,2,3,0,3,1,4,5,7,4,7,6,0,1,5,0,5,4,2,6,7,2,7,3,0,4,6,0,6,2,1,3,7,1,7,5]);}
  const ground=data.display.ground_z_m;
  const grid=[];for(let n=-4;n<=8;n++){grid.push([n*.1,-.4,ground],[n*.1,.4,ground]);}for(let n=-4;n<=4;n++){grid.push([-.4,n*.1,ground],[.8,n*.1,ground]);}const gridBuffer=makeBuffer(grid,[],gl.LINES);
  const probe=document.createElement('span');probe.style.display='none';canvas.parentElement.append(probe);
  const swatch=document.createElement('canvas');swatch.width=swatch.height=1;const colorContext=swatch.getContext('2d',{willReadFrequently:true});
  function rgba(token){probe.style.color=`var(${token})`;colorContext.clearRect(0,0,1,1);colorContext.fillStyle=getComputedStyle(probe).color;colorContext.fillRect(0,0,1,1);return [...colorContext.getImageData(0,0,1,1).data].map(v=>v/255);}
  function rgb(token){const c=rgba(token),bg=rgba('--background');return c.slice(0,3).map((v,i)=>v*c[3]+bg[i]*(1-c[3]));}
  function draw(mesh,matrix,paint){gl.bindBuffer(gl.ARRAY_BUFFER,mesh.buffer);gl.vertexAttribPointer(position,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(normal,3,gl.FLOAT,false,24,12);gl.uniformMatrix4fv(model,false,flatten(matrix));gl.uniform3fv(color,paint);gl.drawArrays(mesh.mode,0,mesh.count);}
  const cad=data.surfaces.display_from_cad;
  const contactMeshes={};
  for(const [key,c] of Object.entries(data.display.contact_triangles||{}))contactMeshes[key]={
    fixed:makeBuffer(c.fixed_vertices,[0,1,2]),moving:makeBuffer(c.moving_vertices_at_contact,[0,1,2])};
  return {render(current,state,project,w,h){
    const ratio=Math.min(2,window.devicePixelRatio||1);canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);gl.viewport(0,0,canvas.width,canvas.height);gl.clearColor(0,0,0,0);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(program);gl.uniformMatrix4fv(projection,false,flatten(project.matrix));
    let neutral=rgb('--muted-foreground'),motor=rgb('--foreground'),left=rgb('--viz-series-1'),right=rgb('--viz-series-3');
    gl.uniform1f(ambient,data.display.inspection_lighting ? .78 : .38);
    if(data.display.inspection_lighting){
      const bg=rgb('--background'),fg=rgb('--foreground'),luma=c=>c[0]*.2126+c[1]*.7152+c[2]*.0722,bright=luma(bg)>luma(fg)?bg:fg;
      const mix=(a,b,t)=>a.map((v,i)=>v*(1-t)+b[i]*t);
      neutral=mix(neutral,bright,.65);motor=mix(neutral,fg,.2);left=mix(rgb('--s2-lift-moving'),bright,.18);
    }
    draw(gridBuffer,I(),rgb('--border'));
    draw(meshes['base_link.STL'],cad,neutral);
    for(const m of data.display.cad_from_wheels)draw(meshes['wheel1.STL'],mul(cad,m),motor);
    let platform=current.platform;
    if(!platform){
      const middle=[0,1,2].map(i=>(current.frames.left_root[i][3]+current.frames.right_root[i][3])/2);
      platform=cad.map(r=>r.slice());
      const ref=data.surfaces.meshes['vertical_link.STL'],center=ref.min.map((v,i)=>v+ref.span[i]/2),xy=point(cad,center);
      platform[0][3]+=middle[0]-xy[0];platform[1][3]+=middle[1]-xy[1];platform[2][3]=middle[2]-data.surfaces.platform_mesh_top_m;
    }
    draw(meshes['vertical_link.STL'],platform,data.display.highlight_platform?left:neutral);
    if(current.contact_key){const m=contactMeshes[current.contact_key],paint=rgb(data.display.inspection_lighting?'--s2-lift-contact':'--orange');draw(m.fixed,cad,paint);draw(m.moving,cad,paint);}
    if(!data.display.lift_only)for(const side of ['left','right'])for(const visual of data.surfaces.arm_visuals){draw(meshes[visual.mesh],mul(current.arms[side][visual.link],visual.origin),visual.material==='motor'?motor:(side==='left'?left:right));}
    const camera=current.frames.camera;
    draw(meshes['d415.stl'],mul(camera,data.camera_model.color_optical_from_mesh),neutral);
  }};
}
// Canvas fallback keeps the official robot geometry visible without WebGL.
function createSoftware(canvas,data,{mul,point}) {
  const ctx=canvas.getContext('2d');if(!ctx)return null;
  const decode=s=>{const raw=atob(s),b=new Uint8Array(raw.length);for(let i=0;i<raw.length;i++)b[i]=raw.charCodeAt(i);return new Uint16Array(b.buffer);};
  const meshes={};
  for(const [name,m]of Object.entries(data.surfaces.meshes)){
    const q=decode(m.vertices),v=[];for(let i=0;i<q.length;i+=3)v.push([0,1,2].map(k=>m.min[k]+q[i+k]/65535*m.span[k]));
    meshes[name]={v,f:decode(m.faces)};
  }
  const probe=document.createElement('span');probe.style.display='none';canvas.parentElement.append(probe);
  const swatch=document.createElement('canvas');swatch.width=swatch.height=1;const color=swatch.getContext('2d',{willReadFrequently:true});
  function rgb(token){probe.style.color=`var(${token})`;color.clearRect(0,0,1,1);color.fillStyle=getComputedStyle(probe).color;color.fillRect(0,0,1,1);return [...color.getImageData(0,0,1,1).data].slice(0,3);}
  return {render(current,state,project,w,h){
    const ratio=Math.min(2,window.devicePixelRatio||1);canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
    const triangles=[],cad=data.surfaces.display_from_cad,fg=rgb('--foreground'),bg=rgb('--background');
    const mix=(a,b,t)=>a.map((v,i)=>v*(1-t)+b[i]*t);
    let neutral=rgb('--muted-foreground'),motor=fg,left=rgb('--viz-series-1'),right=rgb('--viz-series-3');
    if(data.display.inspection_lighting){const luma=c=>c[0]*.2126+c[1]*.7152+c[2]*.0722,bright=luma(bg)>luma(fg)?bg:fg;neutral=mix(neutral,bright,.65);motor=mix(neutral,fg,.2);left=mix(rgb('--s2-lift-moving'),bright,.18);}
    function add(name,m,paint){const mesh=meshes[name],world=mesh.v.map(p=>point(m,p)),projected=world.map(project);
      for(let i=0;i<mesh.f.length;i+=3){const ids=[mesh.f[i],mesh.f[i+1],mesh.f[i+2]],a=world[ids[0]],b=world[ids[1]],c=world[ids[2]],u=b.map((v,k)=>v-a[k]),v=c.map((v,k)=>v-a[k]);
        const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],length=Math.hypot(...n);if(length<1e-14)continue;
        const light=.7+.3*Math.abs((n[0]*.4-n[1]*.6+n[2])/(length*Math.sqrt(1.52)));
        const pts=ids.map(id=>projected[id]);triangles.push({pts,depth:pts.reduce((s,p)=>s+p[2],0)/3,paint:`rgb(${paint.map(v=>Math.round(v*light)).join(',')})`});
      }
    }
    const ground=data.display.ground_z_m;ctx.strokeStyle=getComputedStyle(canvas).getPropertyValue('--border');ctx.lineWidth=.6;
    function line(a,b){a=project(a);b=project(b);ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();}
    for(let n=-4;n<=8;n++)line([n*.1,-.4,ground],[n*.1,.4,ground]);
    for(let n=-4;n<=4;n++)line([-.4,n*.1,ground],[.8,n*.1,ground]);
    add('base_link.STL',cad,neutral);
    for(const m of data.display.cad_from_wheels)add('wheel1.STL',mul(cad,m),motor);
    if(current.platform)add('vertical_link.STL',current.platform,data.display.highlight_platform?left:neutral);
    if(!data.display.lift_only)for(const side of ['left','right'])for(const v of data.surfaces.arm_visuals)add(v.mesh,mul(current.arms[side][v.link],v.origin),v.material==='motor'?motor:side==='left'?left:right);
    add('d415.stl',mul(current.frames.camera,data.camera_model.color_optical_from_mesh),neutral);
    triangles.sort((a,b)=>a.depth-b.depth);
    for(const t of triangles){ctx.beginPath();ctx.moveTo(...t.pts[0].slice(0,2));ctx.lineTo(...t.pts[1].slice(0,2));ctx.lineTo(...t.pts[2].slice(0,2));ctx.closePath();ctx.fillStyle=t.paint;ctx.fill();}
  }};
}
if(typeof module!=='undefined'&&module.exports)module.exports={create};else globalThis.S2MeshRenderer={create};
})();

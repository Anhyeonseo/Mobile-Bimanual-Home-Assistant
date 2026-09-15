/* Dependency-free orthographic 3D occupancy view. Picking uses the rendered
 * surface polygons: a block top or side never falls through to the floor. */
(function(root){
  function inside(point, polygon){
    let hit=false;
    for(let i=0,j=polygon.length-1;i<polygon.length;j=i++){
      const a=polygon[i],b=polygon[j];
      if(((a[1]>point[1])!==(b[1]>point[1])) && point[0]<(b[0]-a[0])*(point[1]-a[1])/(b[1]-a[1])+a[0])hit=!hit;
    }return hit;
  }
  function geometry(map,width,height){
    const w=map.rows[0].length,h=map.rows.length;
    const scale=Math.min((width-40)/(w+h),(height-65)/((w+h)*.46));
    const project=(x,y,z=0)=>[width/2+(x-y-(w-h)/2)*scale,30+(x+y)*scale*.46-z*scale];
    const surfaces=[];
    for(let sum=0;sum<w+h-1;sum++)for(let x=0;x<w;x++){
      const y=sum-x;if(y<0||y>=h)continue;
      const tile=map.rows[y][x],z=tile==='#'?.9:0;
      const p=[project(x,y,z),project(x+1,y,z),project(x+1,y+1,z),project(x,y+1,z)];
      if(z){
        surfaces.push({polygon:[p[1],p[2],project(x+1,y+1),project(x+1,y)],color:'#66816c',blocked:true});
        surfaces.push({polygon:[p[2],p[3],project(x,y+1),project(x+1,y+1)],color:'#78927a',blocked:true});
      }
      surfaces.push({polygon:p,color:tile==='#'?'#a3b59b':tile==='?'?'#c1c7bf':(x+y)%2?'#e0e9d9':'#e7eee0',blocked:tile!=='.',x,y});
    }
    return {surfaces,project,scale};
  }
  function pick(geometry,px,py){
    for(let i=geometry.surfaces.length-1;i>=0;i--){const s=geometry.surfaces[i];if(inside([px,py],s.polygon))return s.blocked?null:{x:s.x,y:s.y};}
    return null;
  }
  function world(map,cell){return {x:map.origin.x+(cell.x+.5)*map.resolution_m,y:map.origin.y+(cell.y+.5)*map.resolution_m,z:map.floor_z_m};}
  const api={inside,geometry,pick,world};root.AlohaMap=api;
  if(typeof module!=='undefined')module.exports=api;
})(typeof window!=='undefined'?window:globalThis);

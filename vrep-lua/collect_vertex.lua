--require './context'

function transVex(vertices, matrix)
   -- return the position relate to the trans matrix
   for i=1,#vertices/3,1 do
      v={vertices[3*(i-1)+1],vertices[3*(i-1)+2],vertices[3*(i-1)+3]}
      v=sim.multiplyVector(matrix,v)
      vertices[3*(i-1)+1]=v[1]
      vertices[3*(i-1)+2]=v[2]
      vertices[3*(i-1)+3]=v[3]
   end
   return vertices
end

function getVertex()
   verteces={}
   faces={}
   local i=1
   while obstacle_handles[i]~=nil do
      local v,f=sim.getShapeMesh(obstacle_handles[i]);
      local m=sim.getObjectMatrix(jhandle[1],obstacle_handles[i])
      --print(m)
      sim.invertMatrix(m)
      --print(m)
      v=transVex(v,m)
      table.insert(verteces,#v)
      table.insert(faces,#f)
      for j=1,#v,1 do
	 table.insert(verteces,v[j])
      end
      for j=1,#f,1 do
	 table.insert(faces,f[j])
      end
      i=i+1
   end
   DH_dummy_handles={-1,-1,-1,-1,-1,-1,-1}
   for i=1,jointNum+1,1 do
      DH_dummy_handles[i]=sim.getObjectHandle("DH"..i)
   end
   for i=1,#link_handles,1 do
      local v,f=sim.getShapeMesh(link_handles[i]);
      local m=sim.getObjectMatrix(link_handles[i],DH_dummy_handles[i+1])
      v=transVex(v,m)
      table.insert(verteces,#v)
      table.insert(faces,#f)
      for j=1,#v,1 do
	 table.insert(verteces,v[j])
      end
      for j=1,#f,1 do
	 table.insert(faces,f[j])
      end
   end
   n1=verteces[1]
   n2=verteces[n1+2]
   n3=verteces[n1+n2+3]
   n4=verteces[n1+n2+n3+4]
   n5=verteces[n1+n2+n3+n4+5]
   n6=verteces[n1+n2+n3+n4+n5+6]
   n7=verteces[n1+n2+n3+n4+n5+n6+7]
   n8=verteces[n1+n2+n3+n4+n5+n6+n7+8]
   n9=verteces[n1+n2+n3+n4+n5+n6+n7+n8+9]
   print({n1,n2,n3,n4,n5,n6,n7,n8,n9})
   print(#verteces)
   sim.setStringSignal("Data_Verteces",sim.packFloatTable(verteces))
   sim.setStringSignal("Data_Faces",sim.packInt32Table(faces))
   sim.setIntegerSignal("getVex_start",0);
   sim.setIntegerSignal("getVex_finish", 1);
end

function init()
   sim.setIntegerSignal("getVex_start",0)
   --getVertex()
end

function actuation()
   SIG_Vex_start=sim.getIntegerSignal("getVex_start");
   if SIG_Vex_start==1 then
      getVertex()
   end
end

--require 'context'

function samplePath(spacenum)
   local position
   local path={}
   for i= 0,spacenum,1 do
      position = i/spacenum;
      sim.setObjectPosition(htarget,-1,sim.getPositionOnPath(hPath, position));
      m = sim.getObjectMatrix(hJoint1,htarget);
      sim.invertMatrix(m)
      path[3*i+1]=m[4]; path[3*i+2]=m[8]; path[3*i+3]=m[12];
   end
   print(path)
   sim.setStringSignal("Data_targetPath",sim.packFloatTable(path))
   sim.setIntegerSignal("SIG_start", 0);
   sim.setIntegerSignal("SIG_finish", 1);
end

function init()
   hJoint1=sim.getObjectHandle(name..'_joint1');
   sim.setIntegerSignal("SIG_start", 0); -- start signal of smaple target path
end

function actuation()
   SIG_start=sim.getIntegerSignal("SIG_start");
   if SIG_start==1 then
      local spacenum=sim.getIntegerSignal("SIG_spacenum")
      print(spacenum)
      samplePath(spacenum);
   end
end

function init()
    hVSensor=sim.getObjectHandle("Vision_sensor")
    sim.setIntegerSignal("SIG_execute",0)
    sim.setIntegerSignal("SIG_execute_end",0)
    sim.setIntegerSignal("SIG_send_sanpshot",0)
    snapshots={}
end

function actuation()
    SIG_execute=sim.getIntegerSignal("SIG_execute")
    SIG_execute_end=sim.getIntegerSignal("SIG_execute_end")
    SIG_send_sanpshot=sim.getIntegerSignal("SIG_send_sanpshot")

    if SIG_execute==1 then
        imageBuffer=sim.getVisionSensorImage(hVSensor)
        --print(#imageBuffer)
        table.insert(snapshots,imageBuffer)
    end
    if SIG_execute_end==1 then
        sim.setIntegerSignal("SIG_execute",0)
        sim.setIntegerSignal("SIG_execute_end",0)
--        print('catch ', #snapshots, ' snapshots')
    end
    if SIG_send_sanpshot==1 then
       print('sends', #snapshots, 'snapshots')
       for i=1,#snapshots,1 do
	  sim.setStringSignal("Data_snapshots",sim.packFloatTable(snapshots[i]))
       end
       sim.setIntegerSignal("SIG_send_sanpshot",2)
    end
end

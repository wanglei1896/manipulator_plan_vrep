function T_para(theta,d,alpha,a)
   local Tran={math.cos(theta),-math.sin(theta)*math.cos(alpha),math.sin(theta)*math.sin(alpha),a*math.cos(theta),
	 math.sin(theta),math.cos(theta)*math.cos(alpha),-math.cos(theta)*math.sin(alpha),a*math.sin(theta),
	 0,math.sin(alpha),math.cos(alpha),d}
   return Tran;
end
function T_para_full(theta, d, alpha, a, ttheta, dd)
   local zTran={math.cos(ttheta),-math.sin(ttheta),0,0,
	  math.sin(ttheta), math.cos(ttheta),0,0,
          0, 0, 1, dd}
   return sim.multiplyMatrices(T_para(theta,d,alpha,a),zTran)
end

-- in case acos() and asin() outOfBounder because numeric precision
function lim_bound(x)
   if x>1 then return 1
   elseif x<-1 then return -1 
   else return x end
end

-- alternate for '=='
function approximate(numA,numB)
   return math.abs(numA-numB)<1e-4
end

function arcsin(sinNum, alterCosNum)
   if approximate(sinNum,0) then
      return math.acos(lim_bound(alterCosNum))
   else
      return math.asin(lim_bound(sinNum))
   end
end
function arccos(cosNum, alterSinNum)
   if approximate(cosNum,0) then
      return math.asin(lim_bound(alterSinNum))
   else
      return math.acos(lim_bound(cosNum))
   end
end

function getParaFromTMatrix(DHT)
   local offset,d,alpha,a,ooffset,dd
--   print(DHT)
   alpha=math.acos(lim_bound(DHT[11]))
   local salpha=math.sin(alpha); local calpha=math.cos(alpha);
   if approximate(salpha,0) then --special case
--    print('salpha is zero')
      a=math.sqrt(DHT[4]^2+DHT[8]^2)
      local sum=arccos(DHT[1],DHT[5])
--      print(math.cos(sum))
      if approximate(a,0) then
--	 print('a is zero')
	 offset=sum
	 ooffset=0
      else
	 offset=arccos(DHT[4]/a,DHT[8]/a)
	 ooffset=(sum-offset)*calpha
      end
      d=DHT[12]
      dd=0
   else --general case
--      print('salpha not zero')
      offset=arcsin(DHT[3]/salpha,-DHT[7]/salpha)
      stheta=math.sin(offset); ctheta=math.cos(offset);
      ooffset=arcsin(DHT[9]/salpha,DHT[10]/salpha)
      if approximate(stheta,0) then --special case
--	 print('stheta is zero')
	 a=DHT[4]/ctheta
	 dd=-DHT[8]/(ctheta*salpha)
	 d=DHT[12]-dd*calpha
      elseif approximate(ctheta,0) then --special case
--	 print('ctheta is zero')
	 a=DHT[8]/stheta
	 dd=DHT[4]/(stheta*salpha)
	 d=DHT[12]-dd*calpha
      else --general case
--	 print('stheta or ctheta not zero')
	 dd=(DHT[4]/ctheta-DHT[8]/stheta)/(ctheta/stheta+stheta/ctheta)/salpha;
	 d=DHT[12]-dd*calpha
	 a=(DHT[8]+dd*ctheta*salpha)/stheta
      end
   end
   local constructDHT=T_para_full(offset, d, alpha, a, ooffset, dd)
--   print(constructDHT)
   for i=1,#DHT,1 do
      if not approximate(DHT[i],constructDHT[i]) then
         print(i,'something wrong')
	 print(DHT[i], constructDHT[i])
      end
   end   
   return offset,d,alpha,a,ooffset,dd
end

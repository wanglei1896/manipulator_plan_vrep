% 用于探索轨迹编码参数的改变对生成的轨迹及其代价函数值的影响
function testParaEffect()

paraSample = rand(1,14);
result=fitnessf(paraSample);
end
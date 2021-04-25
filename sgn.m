function rst = sgn(t,T)
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
if t<T/4
    rst=1;
elseif t>=T/4
    rst=-1;
end
end


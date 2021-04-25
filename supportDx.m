function rst = supportDx(s,t,T)
%UNTITLED6 此处显示有关此函数的摘要
%   此处显示详细说明
rst=s*(2*(T-t)/T+1/(2*pi)*sin(4*pi*t/T));
end


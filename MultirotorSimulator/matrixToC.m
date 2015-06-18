function out = matrixToC(in)
% A script to convert a matrix in MATLAB to C-code representation.
n = size(in,1);
m = size(in,2);
row = 0;
r = 0;
out = '{';

for i=1:n
    row = in(i,:);
    out = strcat(out,'{');
    for j=1:m
       r = row(1,j);
       if (j ~= m)
           out = strcat(out,num2str(r,'%1.10f'),',');
       else
           out = strcat(out,num2str(r,'%1.10f'));
       end
    end
    if(i ~= n)
        out = strcat(out, '},\n');
    else
        out = strcat(out,'}};');
        sprintf(out)
    end
end


end
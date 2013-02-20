n = 1000000;


for i = 1:n
    rand{i} =  normrnd(0,10,1,1);
end
sum = 0;
for i = 1:n
    sum = sum + rand{i};
end

media = sum/n

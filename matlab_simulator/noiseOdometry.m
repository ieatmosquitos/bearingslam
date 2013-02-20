function [ outData ] = noiseOdometry( data,mu,sigma )
%NOISEODOMETRY Given a cell of vectors
%return a cell in where is summed to each vector a normal gaussian noise
%with parameters mu (usually 0) and sigma

nCell = size(data,2);
outData = cell(nCell);

for i = 1:nCell
    vector = data{i};
    sizeVector = size(vector,1);
    normalNoise = normrnd(mu,sigma,sizeVector,1);
    outData{i} = vector+normalNoise;    
end

end


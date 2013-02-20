function [ outData ] = applyNoiseData( data,mu,sigma,ciclic)
%applyNoiseData Given a cell 'data' of vectors
%return a cell in where is summed to each vector a normal gaussian noise
%with parameters mu (usually 0) and sigma
% if 'ciclic' is not 0, noise is applied in a ciclic way (for example to an angle)
% where 'ciclic' is the max value (min value is considered 0)

nCell = size(data,2);
outData = cell(1,nCell);

for i = 1:nCell
    vector = data{i};
    sizeVector = size(vector,1);
    normalNoise = normrnd(mu,sigma,sizeVector,1);
    
    outData{i} = vector + normalNoise;
    
    if ciclic ~= 0
        normalNoise = mod(normalNoise,ciclic); % MOD oppure REM!?!?
    end
    
    % disp(outData{i}); & DEBUG

                    
end

end


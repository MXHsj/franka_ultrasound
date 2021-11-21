%% compute angle between two vectors
function theta = angleBetweenVectors(u,v)

len = length(u);
theta = zeros(len,1);

for i = 1:len
    % cosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1,-1));
    cosTheta = dot(u(i,:),v(i,:))/ ...
               (norm(u(i,:))*norm(v(i,:)));
    theta(i) = real(acos(cosTheta));
end
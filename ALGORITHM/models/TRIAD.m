function ROT_MAT = TRIAD(p1,s1,p2,s2) % function to get rotation matrix in the triad frame

p1 = p1/norm(p1);
p2 = p2/norm(p2);
s1 = s1/norm(s1);
s2 = s2/norm(s2);

A1 = [p1, cross(p1,s1)/norm(cross(p1,s1)), zeros(3,1)];
A1(:,3) = cross(A1(:,1),A1(:,2))/norm(cross(A1(:,1),A1(:,2)));

A2 = [p2, cross(p2,s2)/norm(cross(p2,s2)), zeros(3,1)];
A2(:,3) = cross(A2(:,1),A2(:,2))/norm(cross(A2(:,1),A2(:,2)));

ROT_MAT = A2*transpose(A1);

end
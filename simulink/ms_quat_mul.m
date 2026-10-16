function q = ms_quat_mul(a,b)
%MS_QUAT_MUL Hamilton product of scalar-first column quaternions.
a=a(:); b=b(:);
q=[a(1)*b(1)-dot(a(2:4),b(2:4)); ...
   a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end

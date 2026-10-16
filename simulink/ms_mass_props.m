function [M, cg, Ixx, Iyy, Izz, I, Idot] = ms_mass_props(P, m_f, m_o, mdot)
%MS_MASS_PROPS Full body inertia about instantaneous CG, and its derivative.
% Tank contents are point masses at fixed body locations (no slosh).
% Optional mdot = [fuel mass rate; oxidizer mass rate] in kg/s.
if nargin < 4, mdot = zeros(2, 1); end
m = [P.m_car; P.m_dry_prop; max(m_f,0); max(m_o,0)];
r = [P.cg_car_off; P.dry_pos; P.fuel_pos; P.ox_pos];
M = sum(m); cg = (m.'*r)/M;
I = diag(P.I_scale(:).*P.m_car.* ...
    [P.W^2+P.H^2; P.L^2+P.H^2; P.L^2+P.W^2]/12);
Idot = zeros(3,3); dm = [0; 0; mdot(:)];
for k = 1:4
    d = (r(k,:)-cg).';
    J = dot(d,d)*eye(3)-d*d.';
    I = I + m(k)*J;
    % Moving-CG terms sum to zero since sum(m_k*(r_k-cg))=0.
    Idot = Idot + dm(k)*J;
end
Ixx = I(1,1); Iyy = I(2,2); Izz = I(3,3);
end

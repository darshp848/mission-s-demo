function wind=ms_wind(P,pos)
%MS_WIND Stationary spatial gust field (world m/s), not sensor truth.
phase=2*pi*pos(1)/P.wind_wavelength;
wind=P.wind_mean+P.wind_gust.*[sin(phase);sin(.73*phase+.4);sin(1.31*phase-.2)];
end

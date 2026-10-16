function L=ms_log_matrix(raw,P)
%MS_LOG_MATRIX Normalize Simulink Array logging for column-vector signals.
% Array format preserves a 2D column signal as width x 1 x samples.
c=ms_log_columns(P);
if ismatrix(raw) && size(raw,2)==c.width
    L=raw;
elseif size(raw,1)==c.width && size(raw,2)==1
    L=reshape(raw,c.width,[]).';
else
    error('MissionS:LogShape','Unexpected telemetry shape %s; width %d required',mat2str(size(raw)),c.width);
end
end

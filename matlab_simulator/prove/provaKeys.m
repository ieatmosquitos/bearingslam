figh = figure('name','press some keys') ;
set(figh,'windowkeypressfcn','set(gcbf,''Userdata'',get(gcbf,''CurrentCharacter''))') ;
set(figh,'windowkeyreleasefcn','set(gcbf,''Userdata'','''')') ;
t0 = clock ;
fprintf('\nKeys pressed in 5 seconds:\n') ;
pause ;
while etime(clock,t0) < 5,
    curkey = get(figh,'userdata') ;
    if ~isempty(figh),
        fprintf('%c',curkey) ;
    end
    pause(.05) ;
end
fprintf('\n') ;
delete(figh) ;
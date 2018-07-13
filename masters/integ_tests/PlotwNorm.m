count1 = 0;
wn = zeros(length(w_out.signals.values), 1 );
for r = 1 : length(w_out.signals.values)
	wn(r) = norm( w_out.signals.values( r, : ) );
    if ( wn(r) == 1 )
		disp(r)
		count1 = count1 + 1;
	end
end

disp(count1)
plot(wn);
title('Norm(\omega)');
xlabel('Sample #');
ylabel('Norm');
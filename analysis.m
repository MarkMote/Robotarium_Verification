q = Exp.Sval; 
clear a b 
figure(1)
xlabel('Experiments')
ylabel('Mean Safety Score')
figure(2)
xlabel('Experiments')
ylabel('Variance')
for i = 1:length(q)
    figure(1)
    hold on 
    a = mean(q(1:i));
    plot(i,a,'b.')
    figure(2)
    hold on 
    b = var(q(1:i));
    plot(i,b,'r.')
end


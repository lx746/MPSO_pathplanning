iter=100;
l=0;
w_max=0.9;
w_min=0.4;
r=zeros(1,iter+1);
w=zeros(1,iter);
r(1) = rand();  
% for l=1:iter
    while l<iter
    r(l+2)=4*r(l+1).*(1-r(l+1));
    w(l+1)=w_min*r(l+1)+(w_max-w_min)*l/iter;
    l=l+1;
    figure(10);
    plot(1:iter,w(1:iter),'r-','LineWidth',1.5);
    xlabel('Generation');
    ylabel('Inertia Weight(w)');
    title('Inertia Weight Curve');
    grid on;
    end
% end
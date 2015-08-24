function taylortest
syms x
t = 1:.1:10;
f = sin(x);
fun1 = inline(f);  
sys1 = fun1(t);

plot(t,sys1,'linewidth',2)
grid on

for i=1:20
    
    fun2 = taylor(f,x,0,'order',i);
    o = inline(fun2);
    sys2 = o(t);
    
    hold on
    plot(t,sys2,'color','red','linewidth',.5);
    ylim = ([0 , 5])
    
    pause
end
hold off

end
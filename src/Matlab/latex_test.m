x = linspace(0, 2*pi, 100);
y = sin(x);
plot(x, y);
title('Example Plot');
xlabel('x');
ylabel('sin(x)');

matlab2tikz('myFigureWithLabels.tikz', 'width', '0.8\linewidth');
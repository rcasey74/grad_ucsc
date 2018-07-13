% sinc function approximation, y = sink(x)

% create a sample interval over which we'll be using the function, our
% domain of operation, per se
x = [ 0 : 0.01/1000 : 0.01 ];

% begin with the standard function
y = sinc(x);

% create several polynomial approximations of different order
% 2nd
p2 = polyfit( x, y, 2 );

% evaluate over the domain the interpolated polynomial values
sink2 = p2(1).*x.*x + p2(2).*x + p2(3);
mean( y - sink2 )

% 3rd
p3 = polyfit( x, y, 3 );

% evaluate over the domain the interpolated polynomial values

% 4th
p4 = polyfit( x, y, 4 );

% evaluate over the domain the interpolated polynomial values

% 5th
p5 = polyfit( x, y, 5 );

% evaluate over the domain the interpolated polynomial values

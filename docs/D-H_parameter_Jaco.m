D1 = 0.2755;
D2 = 0.41;
D3 = 0.2073;
D4 = 0.0743;
D5 = 0.0743;
D6 = 0.1687;
e2 = 0.0098;

aa = (11.0*pi)/72;
ca = cos(aa);
sa = sin(aa);
c2a = cos(2*aa);
s2a = sin(2*aa);
d4b = D3 + sa/s2a * D4;
d5b = sa/s2a * D4 + sa/s2a * D5;
d6b = sa/s2a * D5 + D6;

alpha = [pi/2, pi, pi/2, 2*aa, 2*aa, pi]
a     = [0, D2, 0, 0, 0, 0]
d     = [D1, 0, -e2, -d4b, -d5b, -d6b]
q     = sym('q', [1 6]);

% cos(q)  -sin(q)*cos(alpha)   sin(q)*sin(alpha)  a*cos(q)
% sin(q)   cos(q)*cos(alpha)  -cos(q)*sin(alpha)  a*sin(q)
%     0           sin(alpha)          cos(alpha)        d
%     0                   0                   0         1
%

for i=1:1:6
    sinAlpha = round(sin(alpha(i)),3);
    cosAlpha = round(cos(alpha(i)),3);
    A(:,:,i) = [cos(q(i)), -sin(q(i))*cosAlpha,  sin(q(i))*sinAlpha a(i)*cos(q(i)); ...
                sin(q(i)),  cos(q(i))*cosAlpha, -cos(q(i))*sinAlpha a(i)*sin(q(i)); ...
                0, sinAlpha, cosAlpha, d(i); ...
                0, 0, 0, 1];
end

for i=1:1:6
    for j=1:1:4
        for k=1:1:4
            if ~(isa((A(k,j,i)),'sym'))
                if (A(k,j,i) < 0.001)
                    A(k,j,i) = 0;
                end
            end
        end
    end
end

T = A(:,:,1) * A(:,:,2) * A(:,:,3) * A(:,:,4) * A(:,:,5) * A(:,:,6);
simplify(T)

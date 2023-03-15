syms w1 w2 x1 x2
comblin = w1*x1+w2*x2;

sigm0 = 1 - 1/(1+exp(-comblin));
sigm1 = 1/(1+exp(-comblin));

policy0 = log(sigm0);
policy1 = log(sigm1);

gradw0 = [diff(policy0,w1),diff(policy0,w2)];

gradw1 = [diff(policy1,w1),diff(policy1,w2)];

w = [0.8,1];
alpha = 0.1;
r1 = 0;
r2 = 1;
x_0 = [1,0];
x_1 = [1,0];
x_2 = [0,1];
gamma = 0.9;
G1 = r1 + gamma*r2;
w_new = w + alpha*gradw0*G1; 
w_new = subs(w_new,[w1, w2, x1, x2],[w(1),w(2),x_0(1),x_0(2)])
w_new = eval(w_new)

G2 = r2;
w_final = w_new + alpha*gradw1*G2;
w_final = subs(w_final,{w1,w2,x1,x2},{w_new(1),w_new(2),x_1(1),x_1(2)})
w_final = eval(w_final)

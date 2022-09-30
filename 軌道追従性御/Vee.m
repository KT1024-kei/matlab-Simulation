function w=Vee(W)

a = W(1);
b = W(2);
c = W(3);

w = [0 -c  b;
     c  0 -a;
     -b a  0];
 
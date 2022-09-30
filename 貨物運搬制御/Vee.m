function  vee = Vee(vector3)
a1 = vector3(1);
a2 = vector3(2);
a3 = vector3(3);

vee = [ 0   -a3    a2;
        a3   0    -a1;
       -a2   a1    0];
   
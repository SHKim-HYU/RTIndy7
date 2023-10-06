gcc -fPIC -shared -O3 indyrp2_C.c -o indyrp2_C.so -lm
gcc -fPIC -shared -O3 indyrp2_G.c -o indyrp2_G.so -lm
gcc -fPIC -shared -O3 indyrp2_M.c -o indyrp2_M.so -lm
gcc -fPIC -shared -O3 indyrp2_Minv.c -o indyrp2_Minv.so -lm
gcc -fPIC -shared -O3 indyrp2_fd.c -o indyrp2_fd.so -lm
gcc -fPIC -shared -O3 indyrp2_fk.c -o indyrp2_fk.so -lm
gcc -fPIC -shared -O3 indyrp2_fk_ee.c -o indyrp2_fk_ee.so -lm
gcc -fPIC -shared -O3 indyrp2_fkrot_ee.c -o indyrp2_fkrot_ee.so -lm
gcc -fPIC -shared -O3 indyrp2_id.c -o indyrp2_id.so -lm
gcc -fPIC -shared -O3 indyrp2_J_b.c -o indyrp2_J_b.so -lm
gcc -fPIC -shared -O3 indyrp2_J_fd.c -o indyrp2_J_fd.so -lm
gcc -fPIC -shared -O3 indyrp2_J_id.c -o indyrp2_J_id.so -lm
gcc -fPIC -shared -O3 indyrp2_J_s.c -o indyrp2_J_s.so -lm

mv ./*.so ../


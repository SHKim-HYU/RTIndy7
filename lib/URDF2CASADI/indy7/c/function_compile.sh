gcc -fPIC -shared -O3 indy7_C.c -o indy7_C.so -lm
gcc -fPIC -shared -O3 indy7_G.c -o indy7_G.so -lm
gcc -fPIC -shared -O3 indy7_M.c -o indy7_M.so -lm
gcc -fPIC -shared -O3 indy7_Minv.c -o indy7_Minv.so -lm
gcc -fPIC -shared -O3 indy7_fd.c -o indy7_fd.so -lm
gcc -fPIC -shared -O3 indy7_fk.c -o indy7_fk.so -lm
gcc -fPIC -shared -O3 indy7_fk_ee.c -o indy7_fk_ee.so -lm
gcc -fPIC -shared -O3 indy7_fkrot_ee.c -o indy7_fkrot_ee.so -lm
gcc -fPIC -shared -O3 indy7_id.c -o indy7_id.so -lm
gcc -fPIC -shared -O3 indy7_J_b.c -o indy7_J_b.so -lm
gcc -fPIC -shared -O3 indy7_J_fd.c -o indy7_J_fd.so -lm
gcc -fPIC -shared -O3 indy7_J_id.c -o indy7_J_id.so -lm
gcc -fPIC -shared -O3 indy7_J_s.c -o indy7_J_s.so -lm

mv ./*.so ../


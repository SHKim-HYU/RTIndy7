gcc -fPIC -shared -O3 indy7_dualarm_C.c -o indy7_dualarm_C.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_G.c -o indy7_dualarm_G.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_M.c -o indy7_dualarm_M.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_Minv.c -o indy7_dualarm_Minv.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_fd.c -o indy7_dualarm_fd.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_fk.c -o indy7_dualarm_fk.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_fk_ee.c -o indy7_dualarm_fk_ee.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_fkrot_ee.c -o indy7_dualarm_fkrot_ee.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_id.c -o indy7_dualarm_id.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_J_fd.c -o indy7_dualarm_J_fd.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_J_id.c -o indy7_dualarm_J_id.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_J_s.c -o indy7_dualarm_J_s.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_J_b.c -o indy7_dualarm_J_b.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_dJ_s.c -o indy7_dualarm_dJ_s.so -lm
gcc -fPIC -shared -O3 indy7_dualarm_dJ_b.c -o indy7_dualarm_dJ_b.so -lm

mv ./*.so ../


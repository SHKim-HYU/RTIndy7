gcc -fPIC -shared -O3 indy12_v2_C.c -o indy12_v2_C.so -lm
gcc -fPIC -shared -O3 indy12_v2_G.c -o indy12_v2_G.so -lm
gcc -fPIC -shared -O3 indy12_v2_M.c -o indy12_v2_M.so -lm
gcc -fPIC -shared -O3 indy12_v2_Minv.c -o indy12_v2_Minv.so -lm
gcc -fPIC -shared -O3 indy12_v2_fd.c -o indy12_v2_fd.so -lm
gcc -fPIC -shared -O3 indy12_v2_fk.c -o indy12_v2_fk.so -lm
gcc -fPIC -shared -O3 indy12_v2_fk_ee.c -o indy12_v2_fk_ee.so -lm
gcc -fPIC -shared -O3 indy12_v2_fkrot_ee.c -o indy12_v2_fkrot_ee.so -lm
gcc -fPIC -shared -O3 indy12_v2_id.c -o indy12_v2_id.so -lm
gcc -fPIC -shared -O3 indy12_v2_J_s.c -o indy12_v2_J_s.so -lm
gcc -fPIC -shared -O3 indy12_v2_J_b.c -o indy12_v2_J_b.so -lm
gcc -fPIC -shared -O3 indy12_v2_dJ_s.c -o indy12_v2_dJ_s.so -lm
gcc -fPIC -shared -O3 indy12_v2_dJ_b.c -o indy12_v2_dJ_b.so -lm

mv ./*.so ../


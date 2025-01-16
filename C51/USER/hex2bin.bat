cd ..
cd OBJ
srec_cat.exe template.hex -Intel -o T5L51.bin -Binary 
echo.
::copy "T5L51.bin" E:\DWIN_SET
copy "T5L51.bin" ..\
copy "T5L51.bin" ..\..\GUI\DWIN_SET
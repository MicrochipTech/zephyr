

KEYNAME=testkey

rm *.pem
rm ../../boards/arm/mec172xmodular_assy6930/support/*.pem
rm /home/jamezaar/ecwork/CPGZephyrDocs/MEC172x/SPI_image_gen/*.pem

# ======================================================================
#openssl ecparam -name secp384r1 -genkey -out testkey.pem
#openssl req -new -key testkey.pem -out testkey_csr.pem
#openssl x509 -req -days 3650 -in testkey_csr.pem -signkey testkey.pem -out testkey_crt.pem

# ======================================================================
# 1) EC private key of P384
# openssl ecparam -name secp384r1 -genkey -noout -out key.pem

# 3) 1) EC private key of P384
#openssl ec -in key.pem -noout -text

# 4) What is input.bin
# 5) How to generate the public key from the private key
#openssl ec -in key.pem -pubout -out pubkey.pem


# Create the key with password:
set ecdsa_key_filename=ec384
set ecdsa_key_filename_pass=ec384
set csr_file=%ecdsa_key_filename_pass%_csr.pem
set crt_file=%ecdsa_key_filename_pass%_crt.pem

openssl ecparam -name secp384r1 -genkey | \
	openssl ec -out ec384.pem -passout pass:ec384 -aes-256-cbc

openssl req -new -key ec384.pem -out ec384_csr.pem \
	-passin pass:ec384 -subj /C=US/ST=NYC/L=Hauppauge/O=MCHP/OU=CPGFW/CN=CEC1712

openssl x509 -req -days 3650 -in ec384_csr.pem \
	-signkey ec384.pem -out ec384_crt.pem -passin pass:ec384

#cp *.pem ../../boards/arm/mec172xmodular_assy6930/support/
#cp *.pem /home/jamezaar/ecwork/CPGZephyrDocs/MEC172x/SPI_image_gen/
#echo "ls -la *.pem"
ls -la *.pem
#echo "ls -la ../../boards/arm/mec172xmodular_assy6930/support/"
#ls -la ../../boards/arm/mec172xmodular_assy6930/support/

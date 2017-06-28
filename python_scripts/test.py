import paramiko

host = 'robo1vm1.cse.msu.edu'
user = 'simongle'
keyfilename = '/home/simongle/.ssh/id_rsa'
cmd_str = """xterm -title 'Update Script' -hold -e '
		echo \"Login Succesful!\"
		'&
		"""
k = paramiko.RSAKey.from_private_key_file(keyfilename)
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(hostname=host, username=user, pkey=k)
ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(cmd_str)

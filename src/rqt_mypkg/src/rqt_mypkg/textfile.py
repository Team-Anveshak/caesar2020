file1 = open("doc2.txt","r")
print file1.read() 
file1.close
'''import paramiko
import select
client = paramiko.SSHClient()
client.load_system_host_keys()
client.connect('192.168.0.10',username='anveshak',password='anveshak')
transport = client.get_transport()
channel = transport.open_session()
channel.exec_command("cat /caesor2020/doc3.txt")
while True:
  rl, wl, xl = select.select([channel],[],[],0.0)
  if len(rl) > 0:
      # Must be stdout
      print channel.recv(1024)'''
'''
import paramiko 

host = '192.168.0.10'
user = 'anveshak'
secret = 'anveshak'


ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy()) #Set policy to use when connecting to servers without a known host key
ssh.connect(hostname=host, username=user, password=secret)
stdin, stdout, stderr = ssh.exec_command('doc4.txt')
file1 = open("doc4.txt","r")
print file1.read 
'''

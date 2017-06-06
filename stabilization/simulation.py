#!/usr/bin/python
# -*- coding: utf-8 -*-

import subprocess
import json
import time
import os

#Baixa um arquivo da nuvem
def dowloadFile(url, file_name):

    if not os.path.isfile(file_name):
        cmd =  'wget --verbose --auth-no-challenge --no-check-certificate -O ' +\
                file_name + ' ' + url

        args = cmd.split()
        proc = subprocess.Popen(args, stdout=subprocess.PIPE)
        return proc.communicate()
    return None

#Descompacta uma arquivo tar.gz
def unpackFile(compacted_file, file_name):

    if not os.path.isfile(file_name):
        cmd = 'tar -zxvf' + ' ' +  compacted_file + ' ' + file_name
        args = cmd.split()
        proc = subprocess.Popen(args, stdout=subprocess.PIPE)
        return proc.communicate()
    return None

#Remove um arquivo
def removeFile(file_name):
    cmd = 'rm ' + file_name
    args = cmd.split()
    proc = subprocess.Popen(args, stdout=subprocess.PIPE)
    return proc.communicate()

#Escreve um arquivo json
def writeJsonFile(file_name, data):
    with open(file_name, 'w') as file:
        json.dump(data, file)

#Lê arquivo de configuração json
def readJsonFile(file_name):
    with open(file_name, 'r') as inputfile:
        return json.load(inputfile)

#Executa a aplicação para estabilizar um dado video
def executeVideoEstablization(file_path, file, nt):
    cmd = file_path + ' ' + file + ' ' + str(nt)  if nt  else file_path + ' ' + file
    args = cmd.split()
    proc = subprocess.Popen(args, stdout=subprocess.PIPE)
    return proc.communicate()

def main():

    files = readJsonFile('files.json')
    serial_path = 'serial/ser'
    #exec_paths = [('paralelo-pth/', 'paralelo-pth/par'),
    #              ('paralelo-openmp/', 'paralelo-openmp/par'),
    #            ]

    exec_paths = [('paralelo-pth', 'paralelo-pth/par')]

    for ex in exec_paths:
        stats = {}
        for key in files:
            stats[key] = {}
            for url, file in zip(files[key][0], files[key][1]):
                dowloadFile(url, file)
                stats[key][file] = {}
                exec_info = {"parallel":[0, 0, 0, 0, 0],
                             "speedup": [0, 0, 0, 0, 0], "efficiency": [0, 0, 0, 0, 0]
                            }
                #executa aqui a versao sequencial
                stddata = executeVideoEstablization(serial_path , file, 0)
                serial_time = float(stddata[0].split()[-1].strip())

                #executa a chamadas a o metodo de estabilizacao de video para
                # 2, 4, 8, 16 e 32 threads
                for i, nt in enumerate([2, 4, 8, 16 , 32]):
                    stddata = executeVideoEstablization(ex[1], file, str(nt))
                    parallel_time = float(stddata[0].split()[-1].strip())
                    exec_info['parallel'][i] = parallel_time
                    exec_info['speedup'][i] = speedup = (serial_time / parallel_time)
                    exec_info['efficiency'][i] = speedup / float(nt)

                exec_info['serial'] = serial_time
                stats[key][file]['metrics'] = exec_info
                print stats
                removeFile(file)
        writeJsonFile(ex[0]+'/'+ex[0].split()[-1]+'.json', stats)

        #Aqui salvar o json no arquivo

    #    break
    """
    #Download do arquivo
    cmd =  'wget --verbose --auth-no-challenge --no-check-certificate -O '
    file_name = 'birds-hq.mp4'
    url = 'https://www.dropbox.com/s/zzgjid3fcjhcx6r/birds-hq.mp4?dl=0'
    #executa comando


    #Extracao do aquivo
    cmd = 'tar -zxvf animals.tar.gz animals-hq.mp4'
    #executa comando
    cmd = 'rm '
    """

if __name__ == '__main__':
    main()

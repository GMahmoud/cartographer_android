#!/usr/bin/env python
import sys
import getopt

def main(argv):
    inp = ''
    outp = ''
    tag = ''
    force = 'a' 
    try:
        opts, args = getopt.getopt(
            argv, "i:o:w:f", ["input=", "output=", "word=" ])
    except getopt.GetoptError:
        print ('filterlog.py -i <input> -o <output>  <word> -f')
        sys.exit(2)
        
    for opt, arg in opts:
        if opt == '-h':
            print ('filterlog.py -i <input> -o <output> <word> -f')
            sys.exit()
        elif opt in ("-i", "--input"):
            inp = arg
        elif opt in ("-o", "--output"):
            outp = arg
	elif opt in ("--word"):
            tag = arg
            #print arg
	elif opt in ("-f"):
            force = 'w'

    if  inp == '':
        print ('filterlog.py -i <input> -o <output> <word> -f')
        sys.exit()
    if  tag == '':
        print ('filterlog.py -i <input> -o <output> <word> -f')
        sys.exit()
    if outp == '':
        print ('filterlog.py -i <input> -o <output> <word> -f')
        print ('Warning: the output file is named output.txt')  
        #sys.exit()
        outp = 'output.txt'
    f = open(inp)
    f1 = open(outp, force)
    
    print ("Looking for '" + tag +"'")
    if force == 'w' :
	print('Warning: The '+outp+ ' file is going to be deleted if there is any !!')
    else :
        print('Warning: If the '+outp+ ' file exists, we can not delete it, force the script (-f) or delete the file') 
    doIHaveToCopyTheLine=False

    for line in f.readlines():
    	#print (line)
    	doIHaveToCopyTheLine=False
    	if tag in line:
        	doIHaveToCopyTheLine=True
        #if ('I native' in line) or ('W native' in line) or ('E native' in line) or ('F native' in line):
        	#doIHaveToCopyTheLine=True
    	#print (doIHaveToCopyTheLine)
    	if doIHaveToCopyTheLine:
        	f1.write(line)
	
    f1.close()
    f.close()
    print ("Success ")

if __name__ == "__main__":
    main(sys.argv[1:])

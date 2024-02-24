from type1e import Type1E

def Type1E_test(plctype, ip, port):
    pyplc = Type1E(plctype)
    pyplc.connect(ip, port)
    # check batch access to word units
    pyplc.batchwrite_wordunits("D198", [0, 1000, -1000])
    value = pyplc.batchread_wordunits("D198", 3)
    print(value)

    # check batch access to bit units
    # odd size test
    pyplc.batchwrite_bitunits("M220", [0])
    value = pyplc.batchread_bitunits("M220", 1)
    print(value)

    #even size test
    pyplc.batchwrite_bitunits("M220", [1, 0, 1, 0])
    value = pyplc.batchread_bitunits("M220", 4)
    print(value)

    #test random bit access
    # pyplc.randomwrite_bitunits(["M220", "M225", "M230"], [1, 1, 1])
    pyplc.randomwrite_wordunits(['D198', 'D200', 'D202'],[12,-13,15])
    value = pyplc.batchread_wordunits("D198",6)
    print(value)
    # pyplc.remote_stop()
    # pyplc.remote_run()

if __name__ == "__main__":
    plctype, ip, port = ['F','192.168.0.250',8000]
    Type1E_test(plctype, ip, port)
%Gold Code Generation

def shift(register, feedback, output):
    GPS Shift Register
    
    :param list feedback: which positions to use as feedback (1 indexed)
    :param list output: which positions are output (1 indexed)
    :returns output of shift register:
    
    """
    
    # calculate output
    out = [register[i-1] for i in output]
    if len(out) > 1:
        out = sum(out) % 2
    else:
        out = out[0]
        
    # modulo 2 add feedback
    fb = sum([register[i-1] for i in feedback]) % 2
    
    # shift to the right
    for i in reversed(range(len(register[1:]))):
        register[i+1] = register[i]
        
    # put feedback in position 1
    register[0] = fb
    
    return out

# example:
print shift(G1, [3,10], [10])
print G1


function y = shift(register, feedback, output):
    %adapted from https://natronics.github.io/blag/2014/gps-prn/
    for i = 1 : length(output)
        out = register(i);    
    end
    
    if len(out) > 1
        out = mod(sum(out), 2); %CHECK HERE
    else
        out = out(0);
    end    
    
    % modulo 2 add feedback
    for i = 1 : length(feedback)
        fb = fb + register(i);
    end
    fb = mod(fb);
    
    % shift to the right
    for i in reversed(range(len(register[1:]))):
        register[i+1] = register[i]
    
end
    
    
    
    
    
    
    
    
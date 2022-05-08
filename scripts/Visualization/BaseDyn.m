classdef  BaseDyn < handle
    properties (Abstract,SetAccess=private, GetAccess=public)
        dt      
        qsize
        xsize
        usize
        ysize
    end
    
    methods
       % All member functions below implement nothing
        [x_next, y]     = dynamics(Ph, x, u, mode, varargin) 
        dynInfo         = dynamics_par(Ph, x, u, mode,varargin)
        [x_next, y]     = resetmap(Ph, x, mode,varargin) 
        Px              = resetmap_par(Ph, x, mode,varargin)         
        Initialize_model(Ph, varargin)                        
    end
end
classdef PlanarQuadruped < BaseDyn
    properties (SetAccess=private, GetAccess=public)
        dt
        qsize = 7; 
        xsize = 14;
        usize = 4; 
        ysize = 4;          
        S = [zeros(3, 4);
             eye(4)];
        e = 0;  % elastic coefficient        
    end
    
    properties % link geometry and inertia parameters
        bodyMass
        bodyLength
        bodyHeight
        bodyWidth
        bodyCoM
        bodyRotInertia
        hipLinkLength
        hipLinkMass
        hipLinkCoM
        hipRotInertia
        hipLoc
        kneeLoc
        kneeLinkLength
        kneeLinkMass
        kneeLinkCoM
        kneeRotInertia   
        robotMass
    end
    
    properties % robot model (spatial v2)
        model
    end
            
    methods % constructor
        function Quad = PlanarQuadruped(dt)
            Quad.dt = dt;
        end
    end
    
    methods 
        function buildModel(Quad)
            % This function creates a full planar quadruped model structure
            % The 0-configuration is with legs straight down, cheetah
            % pointed along the +x axis of the ICS.
            % The ICS has +z up, +x right, and +y inner page
            % Planar model has 7 DoFs, x, z translation, rotation around y
            % and front (back) hip and knee rotations
            %% model initilization
            robot.NB = 7;                                  % number of moving bodies (2 fictitious bodies for trunk translations)
            robot.parent  = zeros(1,robot.NB);             % parent body indices
            robot.Xtree   = repmat({eye(6)},robot.NB,1);   % coordinate transforms
            robot.jtype   = repmat({'  '},robot.NB,1);     % joint types
            robot.I       = repmat({zeros(6)},robot.NB,1); % spatial inertias
            robot.gravity = [0 0 -9.81]';              % gravity acceleration vec
            
            nb = 0;                                        % current body index
            
            %% trunk translation x (body num 1) (massless)
            nb = nb + 1;
            robot.parent(nb) = nb - 1;
            robot.Xtree{nb} = eye(1);
            robot.jtype{nb} = 'Px';
            robot.I{nb} = mcI(0, zeros(3,1), zeros(3,3));
            
            %% trunk translation z direction (body num 2) (massless)
            nb = nb + 1;
            robot.parent(nb) = nb - 1;
            robot.Xtree{nb} = eye(1);
            robot.jtype{nb} = 'Pz';
            robot.I{nb} = mcI(0, zeros(3,1), zeros(3,3));
            
            %% trunck rotation about y direction (body num 3)
            nb = nb + 1;
            robot.parent(nb) = nb - 1;
            robot.Xtree{nb} = eye(1);
            robot.jtype{nb} = 'Ry';
            robot.I{nb} = mcI(Quad.bodyMass, Quad.bodyCoM, Quad.bodyRotInertia);
            
            nbase = nb; % floating base index for attaching two children links (hip links)
            NLEGS = 2;
            
            for i = 1:NLEGS
                %% Hip link
                nb = nb + 1;
                robot.parent(nb) = nbase; % parent of the hip link is base
                robot.Xtree{nb} = plux(ry(0), Quad.hipLoc{i});  % translation (half the body length)
                robot.jtype{nb} = 'Ry';
                robot.I{nb} = mcI(Quad.hipLinkMass, Quad.hipLinkCoM, Quad.hipRotInertia);
                
                %% Knee link
                nb = nb + 1;
                robot.parent(nb) = nb - 1; % parent of the knee link is hip link
                robot.Xtree{nb} = plux( ry(0), Quad.kneeLoc);    % translation (length of hip link)
                robot.jtype{nb} = 'Ry';
                robot.I{nb} = mcI(Quad.kneeLinkMass, Quad.kneeLinkCoM, Quad.kneeRotInertia);
            end
            Quad.model = robot;
        end                      
    end        
      
    methods
        function T          = getKinematics(Quad,q)
            % This function computes the homogeneous transformation of each joint-fixed
            % frame w.r.t. Inertia coordinate system (IC)
            % Link index convention: body 1, f hip 2, f knee 3, b hip 4, b knee 5
            % q(1):x  q(2):y  q(3):roty
            % q(4):theta1 hip  q(5):theta2 knee (front leg)
            % q(6):theta1 hip  q(7):theta2 knee (back leg)                       
            
            %% kinematics
            % preallocate to save memory
            T  =  repmat({eye(4)}, [5, 1]);
            
            % body Pos and Orientation w.r.t IC
            T{1}    =  Trans([q(1), 0, q(2)])*RotY(q(3));
            
            % body frame to front hip frame
            T12     =  Trans(Quad.hipLoc{1})*RotY(q(4));
            
            % front hip frame to front knee frame
            T23     =  Trans(Quad.kneeLoc)*RotY(q(5));
            
            % body frame to back hip frame
            T14     =  Trans(Quad.hipLoc{2})*RotY(q(6));
            
            % back hip frame to back knee frame
            T45     =  Trans(Quad.kneeLoc)*RotY(q(7));
            
            % front hip frame w.r.t IC
            T{2}    =  T{1}*T12;
            
            % front knee frame w.r.t IC
            T{3}    =  T{2}*T23;
            
            % back hip frame w.r.t IC
            T{4}    = T{1}*T14;
            
            % back knee frame w.r.t IC
            T{5}    = T{4}*T45;
        end
        
        function ICSPos     = getPosition(Quad,q,linkidx, contactLoc)
            % This function gets the position of a contact point w.r.t. ICS.
            % ContactLoc is defined in local frame
            % Link index convention: body 1, f hip 2, f knee 3, b hip 4, b knee 5,                                   
            T  = Quad.getKinematics(q); ICSPos = [];
            for i = 1:size(contactLoc, 2)
                contactLoc_aug = [contactLoc(1,i), 0, contactLoc(2,i), 1]';
                ICSPos = [ICSPos,T{linkidx}*contactLoc_aug];
            end            
                        
            % remove y component and homogeneuous component
            ICSPos([2,4],:) = [];
        end
        
        function BodyPos  = getPositionBodyFrame(Quad,q,linkidx, contactLoc)
            
            T  = Quad.getKinematics(q); BodyPos = [];
            
            for i = 1:size(contactLoc,2)
                contactLoc_aug = [contactLoc(1,i), 0, contactLoc(2,i), 1]';
                BodyPos = [BodyPos, T{1}\T{linkidx}*contactLoc_aug];
            end
            
            % remove y component and homogeneuous component
            BodyPos([2,4],:) = [];
        end
               
    end
       
end
function build2DminiCheetah(Quad)
% This function reformualtes the 3D mc inertia parameter to 2D, and
% creates a full planar quadruped model structure.
% The 0-configuration is with legs straight down, cheetah
% pointed along the +x axis of the ICS.
% The ICS has +z up, +x right, and +y inner page
% Planar model has 7 DoFs, x, z translation, rotation around y
% and front (back) hip and knee rotations

    mc3D = get3DMCParams();
    mc2D = get2DMCParams(mc3D);
    
    Quad.bodyMass            =  mc2D.bodyMass;
    Quad.bodyLength          =  mc2D.bodyLength;
    Quad.bodyHeight          =  mc2D.bodyHeight;
    Quad.bodyWidth           =  mc2D.bodyWidth;
    Quad.bodyCoM             =  mc2D.bodyCoM;
    Quad.bodyRotInertia      =  mc2D.bodyRotInertia;
    Quad.hipLinkLength       =  mc2D.hipLinkLength;
    Quad.hipLinkMass         =  mc2D.hipLinkMass;
    Quad.hipLinkCoM          =  mc2D.hipLinkCoM;
    Quad.hipRotInertia       =  mc2D.hipRotInertia;
    Quad.hipLoc              =  mc2D.hipLoc;
    Quad.kneeLoc             =  mc2D.kneeLoc;
    Quad.kneeLinkLength      =  mc2D.kneeLinkLength;
    Quad.kneeLinkMass        =  mc2D.kneeLinkMass;
    Quad.kneeLinkCoM         =  mc2D.kneeLinkCoM;
    Quad.kneeRotInertia      =  mc2D.kneeRotInertia;
    Quad.robotMass           =  mc2D.robotMass;    
end
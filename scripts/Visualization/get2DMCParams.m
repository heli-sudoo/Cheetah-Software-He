function mc2D = get2DMCParams(mc3D)
mc2D = mc3D;

mc2D.hipLinkMass = mc3D.hipLinkMass * 2;
mc2D.kneeLinkMass = mc3D.kneeLinkMass*2;
mc2D.bodyMass = mc3D.bodyMass + 4*mc3D.abadLinkMass;    % 4 abads in body

% update body CoM (considering 4 abads into body)
weighted_loc = mc3D.bodyMass*mc3D.bodyCoM;
for i = 1:4
    weighted_loc = weighted_loc + mc3D.abadLinkMass*(mc3D.abadLoc{i}+mc3D.abadLinkCoM);
end
mc2D.bodyCoM = weighted_loc/mc2D.bodyMass;

mc2D.hipRotInertia = mc3D.hipRotInertia*2;        % two legs in one
mc2D.kneeRotInertia = mc3D.kneeRotInertia*2;      % two legs in one

mc2D.hipLoc{1} = [mc2D.bodyLength, 0, 0]'/2;
mc2D.hipLoc{2} = [-mc2D.bodyLength, 0, 0]'/2;
mc2D.robotMass = mc2D.bodyMass + 2*mc2D.kneeLinkMass + 2*mc2D.hipLinkMass;
fieldstoRM = {'abadLinkMass', 'abadLinkCoM', 'abadLoc', 'abadLinkLength'};
mc2D = rmfield(mc2D, fieldstoRM);
end
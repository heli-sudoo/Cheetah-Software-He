classdef Graphics < handle
    properties
        quad 
        bodyLength
        bodyHeight
        thighLength
        shankLength
        linkWidth
        trajs = {}
        im
    end
    methods
        function G = Graphics()
            quadParams = get3DMCParams();             
            G.quad = PlanarQuadruped(0.001);
            build2DminiCheetah(G.quad);
            
             %% link data w.r.t. local frame
            G.bodyLength = quadParams.bodyLength+0.04;
            G.bodyHeight = quadParams.bodyHeight+0.04;
            G.thighLength = quadParams.hipLinkLength;
            G.shankLength = quadParams.kneeLinkLength;
            G.linkWidth = 0.03;                                
        end
    end
    
    methods
        function addTrajectory(G, T)
            G.trajs{end+1} = T;
        end
        
        function visualizeTrajectory(G, idx)
            if nargin < 2
                idx = 1;
            end
            if isempty(G.trajs)
                fprintf('No trajectories exisit \n')
                return
            end            
            T = G.trajs{idx};
            
            figure(198);
            ground = drawGround(-0.404);  
            hold on;            
            quadG = G.drawQuad();
            ylim([-1 3]);                       
            axis([-1 3 -1 1]);
            axis equal
            axis manual
            
            % update quadruped graphics
            for k=1:size(T,2)
                pos = T(1:2, k);
                pitch = T(3, k);
                q = T(4:7, k);
                G.updateQuad(quadG, pos, pitch, q);
                pause(0.001);
            end
        end
        
        function compareTrajectory(G, indices)
            if nargin < 2
                indices = [1, 2];
            end
            
            if length(G.trajs) < 2
                fprintf('Not sufficient trajectories exist for comparing \n');
                fprintf('Visualizing the first trajectory \n');
                G.visualizeTrajectory();
                return
            end
            
            Ts= G.trajs(indices);
            num_trajs = length(Ts);
            
            f = figure(200);
            ground = drawGround(-0.404);
            hold on;            
            quadGs = {};
            for i = 1:num_trajs
                quadGs{i} = G.drawQuad();
            end
            ylim([-1 3]);                       
            axis([-1 3 -1 1]);
            axis equal
            axis manual
            
            [~, nc] = cellfun(@size, Ts);
            maxlen = max(nc);
            for k = 1:maxlen
                for i = 1:num_trajs
                    if k <= size(Ts{i}, 2)
                        pos = Ts{i}(1:2,k);
                        pitch = Ts{i}(3,k);
                        q = Ts{i}(4:7,k);
                        G.updateQuad(quadGs{i}, pos, pitch, q);
                    end                    
                end
                pause(0.001);
                im_idx = floor(k/10)+1;
                frame{im_idx} = getframe(f);
                im{im_idx} = frame2im(frame{im_idx});
                im_idx = im_idx + 1;
            end
            G.im = im;            
        end
        
        function save_animation(G)
            filename = 'mc_animation.gif';
            for k = 1:length(G.im)
                [imind,cm] = rgb2ind(G.im{k},256);
                if k == 1
                    imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.05);
                elseif k==length(G.im)
                    imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1);
                else
                    imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.05);
                end
            end
        end
    end
    methods
        function quadGraphics = drawQuad(G)
            % Draw body
            quadGraphics(1) = drawBody2D(G.bodyLength, G.bodyHeight, [0 0.4470 0.7410]);
            % Draw front thigh
            quadGraphics(2) = drawLink2D(G.linkWidth, G.thighLength, [0.7647,0.7686,0.7882]);
            % Draw front shank
            quadGraphics(3) = drawLink2D(G.linkWidth, G.shankLength, [0.7647,0.7686,0.7882]);
            % Draw back thigh
            quadGraphics(4) = drawLink2D(G.linkWidth, G.thighLength, [0.7647,0.7686,0.7882]);
            % Draw back shank
            quadGraphics(5) = drawLink2D(G.linkWidth, G.shankLength, [0.7647,0.7686,0.7882]);
        end
        
        function updateQuad(G, quados, pos, pitch, q)
            C = [pos; pitch; q];
            for idx = 1:5
               v = G.quad.getPosition(C, idx, quados(idx).v');
               set(quados(idx).p, 'vertices', v');
               set(quados(idx).p, 'visible', 'on');
            end
        end
    end     
end
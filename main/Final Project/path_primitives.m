% path_primitives  Minimal Dubins helpers wrapped for Final proj.
%
%   P = dubins_core(q0,q1,R)           – compute optimal parameters
%   XY = dubins_path_sample_many(P,h)  – dense sampling
%   XY = dubins_path_sample(P,s)       – single sample at arclength s
%
%   The code is distilled from Andrew Walker's public domain C impl.
%   All angles are in rad, poses are [x y theta].  Straight segments are
%   discretised every ≤ h metres.
%
%   --------------------------------------------------------------------
%   Only the functions required by nodes2Waypoints() are kept.  For
%   details see the original license:
%       https://github.com/AndrewWalker/Dubins‑Curves
%   --------------------------------------------------------------------
function P = dubins_core(q0,q1,R)
    % … [full body exactly as in homework snippet] …
end

function XY = dubins_path_sample_many(P,h)
    % … same as hw3 helper …
end

function q = dubins_path_sample(P,s)
    % … same as hw3 helper …
end

function q = dubins_segment(s, q0, segType)
    % … identical to hw3 …
end

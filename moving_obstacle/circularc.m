function [xp,yp]=circularc(varargin)
 
%% circularc 
% returns equally spaced points lying on Circular Arc. This function is
% based on the function of [1].
%% Description
% [xp,yp]=CIRCULARC(ARC) computes equally spaced points (xp,yp) lying 
% on circular arc in a counterclockwise direction
% on the current axes where ARC = [Xc Yc Ra Astart Aend] is a row vector
% containing the x- and y- coordinates of the arc center in Xc and Yc,
% the radius of the arc in Ra, the starting angle in Astart, and the ending
% angle in Aend.
%
% [xp,yp]=CIRCULARC(ARC,DIR) computes equally spaced points (xp,yp) lying 
% on circular arc in a counterclockwise direction if DIR='forward', 
% in a clockwise direction if DIR='backward.'
%
% [xp,yp]=CIRCULARC(ARC,DIR,N) tries to computes equally spaced points 
% (xp,yp) lying on circular arc. N is the number of division over a
% circle.
%% References:
%   [1] http://www.mathworks.com/matlabcentral/files/4082/circularc.m
%% Copyright 
%   Karl Ji-Wung Choi (jwchoi@soe.ucsc.edu)
%   $Revision: 1.1.1.1 $  $Date: 2009/02/13 $

rev=2*pi;
arc=varargin{1}(:);
xc=arc(1);
yc=arc(2);
rc=abs(arc(3));
th=rem(arc(4:5),rev);   % wrap angles into one circle
idx=find(th<0);
th(idx)=th(idx)+rev;    % make angles positive, 0 to 2pi
if nargin < 3
    N = 500;
else
    N = varargin{3};
end
if nargin>=2 && strcmpi(varargin{2},'backward')
   if th(1)<th(2)
     th(1)=th(1)+rev;
   end
    np=max(ceil(abs(th(1)-th(2))*N/(2*pi)),2);
else
   if th(2)<th(1)
     th(2)=th(2)+rev;
   end
    np=max(ceil(abs(th(2)-th(1))*N/(2*pi)),2);
end
   
thp=linspace(th(1),th(2),np);
xp=rc*cos(thp)+xc;   % x axis points
yp=rc*sin(thp)+yc;   % y axis points

function saveFigures( hFig, figName )
% ========================================================================
% saveFigures: 
%   - An utility function that saves the figure as an image
%   exp_T = getExpProd( exp_T_arr )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] hFig - A figure handle which we aim to plot
% 
%   [2] figName - The name of the figure
%
% The output will be a pdf file for the image
% ========================================================================

set( hFig, 'Units' , 'Inches' );
pos = get( hFig, 'Position' );
set( hFig, 'PaperPositionMode',   'Auto', ...
                  'PaperUnits', 'Inches', ...
                   'PaperSize', [ pos( 3 ), pos( 4 ) ] )
               
saveas( hFig, figName + ".pdf" )

end


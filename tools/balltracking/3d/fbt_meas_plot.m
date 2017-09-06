function fbt_meas_plot(measurements, varargin)
% plot measurements 


% unpack arrays in measurements
id     = measurements.data(:, fbt_meas_idx('id'));
mt     = measurements.data(:, fbt_meas_idx('t'));
mxc    = measurements.data(:, fbt_meas_idx('xc'));
myc    = measurements.data(:, fbt_meas_idx('yc'));
mzc    = measurements.data(:, fbt_meas_idx('zc'));
mphi   = measurements.data(:, fbt_meas_idx('phi'));
maz    = measurements.data(:, fbt_meas_idx('az'));
mel    = measurements.data(:, fbt_meas_idx('el'));
mr     = measurements.data(:, fbt_meas_idx('r'));
mconf  = measurements.data(:, fbt_meas_idx('conf'));


% settings
options.colors        = fbt_colors;
options.axes          = gca;
options.mode          = 1; % see figure descriptions in fbt_gui and fbt_settings
options.what          = [1 1 1 1 1 1 1]; % fine-tuned control on what to plot
options.whatstyle     = {'x', '+', '*', 's', 'd', 'v', '^'};
options.textlabels    = false;
options.t0            = min(mt);
options.text_y_offset = 0.1;

% parse options
[options, args]       = getopts(options, varargin{:});
assert(numel(args) == 0); % don't know what to do with unparsed arguments

% plot
hold(options.axes, 'on');
nmeas = measurements.n;

% loop over robots for color grouping, for speed (looping over each measurement is way too slow)
for irobot = 1:6
    imeas = find(id == irobot);
    if numel(imeas)
        t    = mt(imeas) - options.t0;
        x    = mxc(imeas);
        y    = myc(imeas);
        z    = mzc(imeas);
        phi  = mphi(imeas);
        az   = maz(imeas);
        el   = mel(imeas);
        r    = mr(imeas);
        conf = mconf(imeas);
        [bx, by, bz] = fct_ball2fcs(x, y, z, phi, az, el, r);
        if options.mode == 1 % 3d scene
            % camera location
            plot3(options.axes, x, y, z, [options.colors(irobot) '*'], 'MarkerSize', 16);
            % robot orientation
            x2 = x + cos(phi);
            y2 = y + sin(phi);
            n = numel(x);
            xx = reshape([x'; x2'], [2*n 1]);
            yy = reshape([y'; y2'], [2*n 1]);
            zz = reshape([z'; z'], [2*n 1]);
            plot3(options.axes, xx, yy, zz, [options.colors(irobot) '-']);
            % ball in RCS
            % no dashed line
            plot3(options.axes, [bx], [by], [bz], [options.colors(irobot) 'o']);
        elseif options.mode == 2
            % ball in FCS
            if options.what(1)
                plot(options.axes, t, bx, [options.colors(irobot) options.whatstyle{1}]);
                sub_textlabel(options, irobot, 'bx', t(end), bx(end));
            end
            if options.what(2)
                plot(options.axes, t, by, [options.colors(irobot) options.whatstyle{2}]);
                sub_textlabel(options, irobot, 'by', t(end), by(end));
            end
            if options.what(3)
                plot(options.axes, t, bz, [options.colors(irobot) options.whatstyle{3}]);
                sub_textlabel(options, irobot, 'bz', t(end), bz(end));
            end
        elseif options.mode == 3
            % raw ball measurements FCS
            if options.what(1)
                plot(options.axes, t, az, [options.colors(irobot) options.whatstyle{1}]);
                sub_textlabel(options, irobot, 'az', t(end), az(end));
            end
            if options.what(2)
                plot(options.axes, t, el, [options.colors(irobot) options.whatstyle{2}]);
                sub_textlabel(options, irobot, 'el', t(end), el(end));
            end
            if options.what(3)
                plot(options.axes, t, r, [options.colors(irobot) options.whatstyle{3}]);
                sub_textlabel(options, irobot, 'r', t(end), r(end));
            end
            if options.what(4)
                plot(options.axes, t, conf, [options.colors(irobot) options.whatstyle{4}]);
                sub_textlabel(options, irobot, 'conf', t(end), conf(end));
            end
            if options.what(5)
                plot(options.axes, t, x, [options.colors(irobot) options.whatstyle{5}]);
                sub_textlabel(options, irobot, 'x', t(end), x(end));
            end
            if options.what(6)
                plot(options.axes, t, y, [options.colors(irobot) options.whatstyle{6}]);
                sub_textlabel(options, irobot, 'y', t(end), y(end));
            end
            if options.what(7)
                plot(options.axes, t, phi, [options.colors(irobot) options.whatstyle{7}]);
                sub_textlabel(options, irobot, 'phi', t(end), phi(end));
            end
        end
    end
end

        
function sub_textlabel(options, irobot, what, tpos, ypos)
    if options.textlabels
        tt = text(tpos, ypos + options.text_y_offset, sprintf('%sR%d', what, irobot));
        set(tt, 'Parent', options.axes);
        set(tt, 'Color', options.colors(irobot));
    end

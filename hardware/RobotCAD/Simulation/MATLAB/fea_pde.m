function T = fea_pde(which, P)
%FEA_PDE  Cross-check of fea.py with the PDE Toolbox (linear static, quadratic tets).
%   fea_pde()            runs all three cases
%   fea_pde("bracket")   or "housing" / "guide"
%   Same parts, materials, loads and mesh sizes as Admin/build_scripts/fea.py,
%   read from Exports/STEP. Units inside: mm, N, MPa.
%
%   One difference: fea.py loads and fixes individual mesh facets picked by
%   position, while the PDE Toolbox works on whole CAD faces. Where fea.py
%   used only part of a face, this script uses the whole face that contains
%   that patch:
%     loads   spread over the whole face (same total force, lower local stress)
%     seats   (vertical-only supports on part of a face) are solved twice:
%             "held" holds the whole face, "free" leaves it out. fea.py's
%             partial seat lies between the two, so the pair brackets it.

if nargin < 1 || which == "", which = "all"; end
if nargin < 2, P = rv_params(); end
G = P.G;
cases = struct("name", {}, "fn", {});
cases(end+1) = struct("name", "bracket", "fn", @bracket);
cases(end+1) = struct("name", "housing", "fn", @housing);
cases(end+1) = struct("name", "guide", "fn", @guide);

rows = {};
for c = cases
    if which == "all" || which == c.name
        rows = [rows; c.fn()]; %#ok<AGROW>
    end
end
T = cell2table(rows, VariableNames=["part" "peak_MPa" "p995_MPa" "max_def_mm" "fs_peak" "fea_py_peak" "fea_py_p995" "fea_py_def"]);
disp(T)

%% ------------------------------------------------------------------ cases
    function r = bracket()
        % RV-434 motor bracket, 3 mm 5052-H32; wheel load 2 g through the 6 gearbox bolts + overhang moment
        Fw = 42.2 * 2;
        dx = 9.0 - 1.5;
        ang = 0:60:300;
        holes = [15.5*cosd(ang); 15.5*sind(ang)]';      % (z, y)
        ys = holes(:,2);
        F = part("RV-434_MotorBracket", 1.2);
        loads = {};
        for k = 1:6
            zc = holes(k,1); yc = holes(k,2);
            ids = F.pick(@(X,Y,Z) X > -0.1 & X < 3.1 & hypot(Z - zc, Y - yc) < 1.9);
            loads(end+1, :) = {ids, [dx*Fw*yc/sum(ys.^2), -Fw/6, 0]}; %#ok<AGROW>
        end
        rail = G.RAIL_Y - G.MOTOR_FACE_Y;
        bolts = F.pick(@(X,Y,Z) hypot(X - rail, abs(Z) - 12) < 2.3 & Y < -18);
        seat = F.touch(@(X,Y,Z) abs(Y - F.ymin) < 0.05, @(X,Y,Z) X > rail - 10 & X < rail + 10);
        fixes = {bolts, "all"; seat, "y"};
        r = run("motor_bracket", F, 70300, 0.33, 193, 3, fixes, loads, [62.2 36.7 0.371], ...
            sprintf("RV-434 5052-H32 3 mm; wheel load %.1f N (2 g) via 6 bolts + overhang moment %.0f N.mm", Fw, dx*Fw));
    end

    function r = housing()
        % RV-308 front caster housing, PETG; carriage bottomed on a 2 g bump pushes up on the top plate
        Fc = max(0.15 * 8.67 * 9.81, 15) * 2;
        F = part("RV-308_FrontCasterHousing", 1.5);
        up = F.touch(@(X,Y,Z) abs(Y - 14) < 0.05, @(X,Y,Z) abs(X) < 22 & abs(Z) < 22);
        zw = G.RAIL_X - G.PROFILE/2 - G.CASTER_X;
        wall = F.pick(@(X,Y,Z) abs(Z - zw) < 0.05);
        r = run("caster_housing", F, 2100, 0.38, 30, 2, {wall, "all"}, {up, [0 Fc 0]}, [5.1 1.6 0.25], ...
            sprintf("RV-308 PETG (E 2.1 GPa, yield 30 MPa across layers); carriage bottomed, %.1f N up (2 g)", Fc));
    end

    function r = guide()
        % RV-433 drawer guide, 1.5 mm 5052 L; tray + battery at 2 g, half per guide
        Fg = (0.280 + 0.900 + 0.021) * 9.81 * 2 / 2;
        F = part("RV-433_DrawerGuide", 1.2);
        top = F.touch(@(X,Y,Z) abs(Y) < 0.05, @(X,Y,Z) X > 48 & X < 63.5 & Z > -208 & Z < 125);
        % fea.py also seats the leg on the cross members (|z| >= 188 mm). That strip is part of the
        % leg's whole underside, which is the face carrying the load, so holding it would hold
        % everything; the guide is held at its bolt holes only (slightly conservative).
        holes = F.pick(@(X,Y,Z) hypot(X - 56, abs(Z) - 198) < 2.3);
        r = run("drawer_guide", F, 70300, 0.33, 193, 3, {holes, "all"}, {top, [0 -Fg 0]}, [19.5 4.3 0.12], ...
            sprintf("RV-433 5052-H32 1.5 mm L; %.1f N per guide (2 g) on the horizontal leg", Fg));
    end

%% ----------------------------------------------------------------- engine
    function F = part(name, h)
        g = fegeometry(fullfile(P.dirs.step, name + ".step"));
        g = scale(g, 1000);                                % STEP comes in as metres; work in mm
        m = femodel(AnalysisType="structuralStatic", Geometry=g);
        m = generateMesh(m, Hmax=h, Hmin=0.3*h, GeometricOrder="quadratic");
        msh = m.Geometry.Mesh;
        X = msh.Nodes;
        tri = freeBoundary(triangulation(double(msh.Elements(1:4,:))', X'));
        nf = m.Geometry.NumFaces;
        fn = cell(nf, 1); area = zeros(nf, 1);
        for k = 1:nf
            fn{k} = findNodes(msh, "region", Face=k);
            in = ismember(tri, fn{k});
            t = tri(all(in, 2), :);
            a = X(:, t(:,1)); b = X(:, t(:,2)); c = X(:, t(:,3));
            area(k) = sum(vecnorm(cross(b - a, c - a), 2, 1)) / 2;
        end
        F.name = name; F.h = h; F.model = m; F.X = X; F.faceNodes = fn; F.area = area;
        F.ymin = min(X(2,:));
        F.pick = @(pred) pickfaces(pred, X, fn);
        F.touch = @(plane, region) touchfaces(plane, region, X, fn);
    end

    function r = run(tag, F, E, nu, sy, fsreq, fixes, loads, ref, desc)
        hasSeat = any(string(fixes(:,2)) == "y");
        variants = "only";
        if hasSeat, variants = ["held" "free"]; end
        lines = ["FEA " + tag + " (PDE Toolbox, MATLAB " + version("-release") + ")"; "  " + desc];
        r = {};
        for vnt = variants
            m = F.model;
            m.MaterialProperties = materialProperties(YoungsModulus=E, PoissonsRatio=nu);
            notes = strings(0);
            for k = 1:size(fixes, 1)
                ids = fixes{k,1};
                if isempty(ids), error("fea_pde:fix", "%s: no faces for fixture %d", tag, k); end
                if fixes{k,2} == "all"
                    m.FaceBC(ids) = faceBC(Constraint="fixed");
                    notes(end+1) = sprintf("  fixed: faces %s, %.0f mm2", mat2str(ids), sum(F.area(ids))); %#ok<AGROW>
                elseif vnt == "held"
                    m.FaceBC(ids) = faceBC(YDisplacement=0);
                    notes(end+1) = sprintf("  seat held vertically over the whole face: faces %s, %.0f mm2", mat2str(ids), sum(F.area(ids))); %#ok<AGROW>
                else
                    notes(end+1) = sprintf("  seat left free (faces %s)", mat2str(ids)); %#ok<AGROW>
                end
            end
            tot = [0 0 0];
            for k = 1:size(loads, 1)
                ids = loads{k,1};
                if isempty(ids), error("fea_pde:load", "%s: no faces for load %d", tag, k); end
                A = sum(F.area(ids));
                m.FaceLoad(ids) = faceLoad(SurfaceTraction=loads{k,2} / A);
                tot = tot + loads{k,2};
                notes(end+1) = sprintf("  load %s N on faces %s (%.0f mm2)", mat2str(loads{k,2}, 3), mat2str(ids), A); %#ok<AGROW>
            end
            S = solve(m);
            vm = S.VonMisesStress;
            um = S.Displacement.Magnitude;
            peak = max(vm); p995 = prctile(vm, 99.5); dmax = max(um);
            verdict = "FAIL";
            if sy/peak >= fsreq, verdict = "PASS";
            elseif sy/p995 >= fsreq, verdict = "PASS (bulk); peak is at a fixture";
            end
            lines = [lines; "  [" + vnt + "]"; notes(:); ...
                sprintf("    mesh: %d nodes, %d quadratic tets, Hmax %.1f mm", size(F.X,2), size(m.Geometry.Mesh.Elements,2), F.h);
                sprintf("    applied load (sum): (%.1f, %.1f, %.1f) N", tot);
                sprintf("    max displacement: %.4f mm", dmax);
                sprintf("    von Mises: peak %.1f MPa (nodal), 99.5th percentile %.1f MPa", peak, p995);
                sprintf("    safety factor on yield %.0f MPa: %.1f (peak), %.1f (99.5 %%) -> %s vs FS >= %g", sy, sy/peak, sy/p995, verdict, fsreq)]; %#ok<AGROW>
            f = figure(Visible="off", Position=[100 100 900 700]);
            s = 0.1 * max(range(F.X, 2)) / max(dmax, eps);
            pdeplot3D(S.Mesh, ColorMapData=vm, Deformation=S.Displacement, DeformationScaleFactor=s);
            clim([0 min(peak, 2*p995)]);
            title(sprintf("%s [%s]: peak %.1f MPa, 99.5 %% %.1f MPa; deformation x%.0f", tag, vnt, peak, p995, s), Interpreter="none");
            sfx = ""; if vnt ~= "only", sfx = "_" + vnt; end
            exportgraphics(f, fullfile(P.dirs.out, "matlab_fea_" + tag + sfx + ".png"), Resolution=110);
            close(f);
            r(end+1, :) = {string(tag) + sfx, peak, p995, dmax, sy/peak, ref(1), ref(2), ref(3)}; %#ok<AGROW>
        end
        lines(end+1) = sprintf("  fea.py (gmsh + scikit-fem, partial seat): peak %.1f MPa, 99.5 %% %.1f MPa, max displacement %.2f mm", ref);
        writelines(lines, fullfile(P.dirs.out, "matlab_fea_" + tag + ".txt"));
        fprintf("%s\n", lines);
    end
end

function ids = pickfaces(pred, X, fn)
ids = [];
for k = 1:numel(fn)
    x = X(:, fn{k});
    if mean(pred(x(1,:), x(2,:), x(3,:))) >= 0.5
        ids(end+1) = k; %#ok<AGROW>
    end
end
end

function ids = touchfaces(plane, region, X, fn)
% faces lying in the plane (>= 90 % of nodes) that contain part of the region
ids = [];
for k = 1:numel(fn)
    x = X(:, fn{k});
    onp = plane(x(1,:), x(2,:), x(3,:));
    if mean(onp) >= 0.9 && any(onp & region(x(1,:), x(2,:), x(3,:)))
        ids(end+1) = k; %#ok<AGROW>
    end
end
end

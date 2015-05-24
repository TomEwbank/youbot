%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANSAC-based cylinder detection function.
%
%[best_detected_cylinder,best_cylinder_points,it] = ransac_cylinder(A)
%
%
% Input: - 6xn matrix A, contains a cloud of oriented points.
% First three numbers in a column are coordinates of the point,
% the other three are its direction unit vector.
% Output: - 7x1 vector best_detected_cylinder,
% the first three numbers are coordinates of one point,
% the next three numbers are coordinates of the second point,
% which together define an axis of the cylinder.
% The seventh number is a radius of the cylinder.
% - 1xn vector best_cylinder_points. If k-th point of A belongs to
% detected cylinder, then k-th element of best_line_points is 1.
% Otherwise it is 0.
% - it is an integer, which shows how many iterations it took to
% find the cylinder.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [best_detected_cylinder,best_cylinder_points,it] = ransac_cylinder(A)
% parameters
dist_tol = 0.005; % in meter, 0.005 default, all features under 2*dist_tol are considered noise.
angle_tol = 15; % in degrees
min_points = 50; % Candidates with size below this value are ignored.
subset_size = 1000; % Maximal size of each subset.
max_it = 10000; % After max_it iterations the search is interrupted.
p_best = 0.99; % A candidate is extracted, as soon as the probability of
% it being the best possible shape is at least p_best
refitting = 1; % Value 1 turns refitting on. Otherwise off.
scatter_mult = 3; % 3 default. Points within dist_tol*scatter_mult from candidates axis are used for refitting.
connectedness = 1; % Value 1 turns LCC on. Otherwise off.
beta = 0.02; %in meter, 0.015 for accurate detection of small gaps, 0.02 for better detection far from the sensor
scale_check = 1; % Value 1 turns on the check, that the radius is within allowed limits. Otherwise off.
min_rad = 0.1; % minimal diameter = 200 mm
max_rad = 0.35; % maximal diameter = 700 mm
max_rad_tol = 1.2; % BEFORE being refitted, a candidate is allowed to have a radius of up to max_rad*max_rad_tol.
% declarations
extract_trigger = 0; % As soon as this value is set to 1, the loop breaks and shape is extracted
size_A = size(A,2);
if size_A<min_points, error('ransac_cylinder: input point cloud too small!'),
end;
best_detected_cylinder = zeros(7,1);
best_cylinder_points = zeros(1,size_A);
best_scores = zeros(3,1);
best_cylinders_points = [];
best_detected_cylinders = [];
largest_cand_size = 0;
angle_tol = cos(angle_tol*pi/180); % the algorithm operates with the cosine of the angle.
number_of_subsets = ceil(size_A/subset_size); % rounding up to avoid getting
number_of_subsets = 0
number_of_points = zeros(2,number_of_subsets); % first row - size of a subset
% second row - size of a subset summed with all previous sizes.
sampling = rand(1,size_A);
id = 1:size_A;
work_samples = sampling < 1/number_of_subsets;
A_work = A(:,work_samples);
id_work = id(work_samples);
number_of_points(:,1) = [1;1]*size(A_work,2);
current_subset = 1;
t_total = tic;
% Starting iterations
it = 0;
while it<max_it
    it = it+1;
    % clearing vectors before each iteration
    acceptable_points = zeros(1,number_of_points(2,current_subset));
    n_dir = zeros(3,number_of_points(2,current_subset));
    dist = zeros(1,number_of_points(2,current_subset));
    
    % picking 3 random columns of A
    sample1_id = randi([1,number_of_points(2,current_subset)]);
    sample2_id = randi([1,number_of_points(2,current_subset)]);
    sample3_id = randi([1,number_of_points(2,current_subset)]);
    % checking, if sample1 and sample2 are the same, to avoid dividing
    % by zero in point_to_line function.
    rand_it = 0;
    while(sample1_id == sample2_id) && rand_it<10
        rand_it=rand_it+1;
        sample1_id = randi([1,number_of_points(2,current_subset)]);
        sample2_id = randi([1,number_of_points(2,current_subset)]);
    end
    if sample1_id == sample2_id
        error('RANSAC_cylinder error: Cannot find 2 different points to build a line candidate')
    end
    
    sample1 = A_work(:,sample1_id);
    sample2 = A_work(:,sample2_id);
    sample3 = A_work(:,sample3_id);
    % defining a candidate cylinder from the first two samples
    [cand_cyl_axis,cand_cyl_rad] = cand_cyl(sample1,sample2);
    rad_ok = 1;
    if scale_check == 1
        if cand_cyl_rad<min_rad || cand_cyl_rad>max_rad*max_rad_tol
            rad_ok = 0;
            %Radius is out of range. Rejecting the candidate.
        end
    end
    % checking, if the candidate cylinder is compatible with all three samples
    [n2,d2] = point_to_line(sample2(1:3),cand_cyl_axis);
    [n3,d3] = point_to_line(sample3(1:3),cand_cyl_axis);
    if abs(d2-cand_cyl_rad)<dist_tol && abs(d3-cand_cyl_rad)<dist_tol...
            && abs(n2'*sample2(4:6))>angle_tol && abs(n3'*sample3(4:6))>angle_tol...
            && rad_ok ==1
        
        % searching for points close to candidate cylinder and storing the
        % result in acceptable_points vector.
        for i=1:number_of_points(2,current_subset)
            [n_dir(:,i),dist(i)] = point_to_line(A_work(1:3,i),cand_cyl_axis);
            if abs(dist(i)-cand_cyl_rad)<dist_tol && abs(A_work(4:6,i)'*n_dir(:,i))>angle_tol
                acceptable_points(i) = 1;
            end
        end
    end
    
    % checking if a candidate fulfills a minimum threshold. If this never
    % happenes, the function will reach maximum iterations and return empty
    % values for output variables.
    
    
    cand_size=sum(acceptable_points);
    if cand_size>=min_points %ceil(min_points/number_of_subsets) this is an alternative, good for finding small cylinders in scans where many points do not lie on cylinders.
        cand_score = score_estimation(size_A,number_of_points(2,current_subset),cand_size);
        % assigning the first candidate
        if best_scores(2,1) == 0
            largest_cand_size = cand_size;
            disp(['First candidate found. Size = ',num2str(cand_size)]);
            best_scores = cand_score
            best_detected_cylinders = [cand_cyl_axis(:,1); cand_cyl_axis(:,2);
                cand_cyl_rad];
            best_cylinders_points = zeros(1,size_A);
            best_cylinders_points(1,id_work(acceptable_points==1)) =1;
            %%%%%%%%%%%%%%%
            % debugging block
            if size(id_work) ~=size(acceptable_points)
                size(id_work)
                size(acceptable_points)
                error('ransac_cylinder: size(id_work) not equal size(acceptable_points)');
            end
            
            
            %%%%%%%%%%%%%%%
        else
            % checking if a candidate is potentially the largest
            %(its trust interval overlaps with the trust interval of the
            candidate with largest expected value)
            if cand_score(3)>=best_scores(1,1)
                if cand_score(2)>=best_scores(2,1) && not_found_yet(best_scores(2,1),best_detected_cylinders(:,1),cand_score(2),[cand_cyl_axis(:,1); cand_cyl_axis(:,2); cand_cyl_rad])
                    % a new best candidate found
                    disp(['New best candidate found. Size = ',num2str(cand_size)]);
                    largest_cand_size = cand_size;
                    best_scores = [cand_score best_scores]
                    best_detected_cylinders =[ [cand_cyl_axis(:,1);
                        cand_cyl_axis(:,2); cand_cyl_rad] best_detected_cylinders];
                    best_cylinders_points = [zeros(1,size_A);
                        best_cylinders_points];
                    best_cylinders_points(1,id_work(acceptable_points==1)) =1;
                elseif not_found_yet(best_scores(2,1),best_detected_cylinders(:,1),cand_score(2),[cand_cyl_axis(:,1); cand_cyl_axis(:,2); cand_cyl_rad])
                    disp(['New candidate found with lower expected size. Size = ',num2str(cand_size)]);
                    best_scores = [best_scores cand_score]
                    best_detected_cylinders =[ best_detected_cylinders
                        [cand_cyl_axis(:,1); cand_cyl_axis(:,2); cand_cyl_rad]];
                    best_cylinders_points = [best_cylinders_points; zeros(1,size_A)];
                    best_cylinders_points(size(best_cylinders_points,1),id_work(acceptable_points==1)) =1;
                end
                %%%%%%%%%%%%%%%
                % debugging block
                if size(id_work) ~=size(acceptable_points)
                    size(id_work)
                    size(acceptable_points)
                    error('ransac_cylnder: size(id_work) not equal size(acceptable_points)');
                end
                %%%%%%%%%%%%%%%
            end
            
        end
    end
    
    % Every 20 iterations it is checked, if the probability is high enough,
    % that there is no larger candidate, than the current largest candidate.
    % As soon as the probability p_best is reached, the algorithm either adds
    % another subsample and continues or returns the found cylinder.
    if mod(it,20) == 0
        p_found = 1 -(1 -(largest_cand_size/number_of_points(2,current_subset))^2)^it;
        if p_found>p_best
            
            %%%%%%%%%%%%%%%%%%
            % Debugging Block
            %%%%%%%%%%%%%%%%%%
            disp(['Probability limit reached after ',num2str(it),' iterations.']);
            disp('Size of detected candidates...');
            disp(sum(best_cylinders_points,2)');
            disp(['...out of ',num2str(number_of_points(2,current_subset))]);
            if current_subset > number_of_subsets
                error('Error: current_subset > number_of_subsets');
            end
            %%%%%%%%%%%%%%%%%
            
            if size(best_scores,2)==1 || current_subset==number_of_subsets
                
                % shape successfully detected!
                
                %%%%%%%%%%%%%%%
                % Message Block
                %%%%%%%%%%%%%%%
                if(size(best_scores,2) ~= size(best_detected_cylinders,2)) || ...
                        (size(best_scores,2) ~= size(best_cylinders_points,1))
                    error('Error: Dimenions mismatch, number of scores and candidates not equal')
                end
                if it == max_it , disp('Max iterations reached'),end
                if current_subset==number_of_subsets, disp('Max subsets reached'),end
                if size(best_scores,2) > 1, disp('More then one candidate found, extracting the largest'),end
                %%%%%%%%%%%%%%
                
                extract_trigger = 1;
            else
                % need another subset
                new_subset_samples = (sampling > current_subset/number_of_subsets)&(sampling <(current_subset+1)/number_of_subsets);
                A_new_subset = A(:,new_subset_samples);
                id_new_subset = id(new_subset_samples);
                current_subset = current_subset+1;
                disp(['New subset added. Current subset = ',num2str(current_subset)]);
                number_of_points(1,current_subset) = size(A_new_subset,2);
                number_of_points(2,current_subset) = number_of_points(2,current_subset-1)+number_of_points(1,current_subset);
                
                
                
                % Test existing candidates on a new subset only
                % -> add to the existing results
                % recalculate trust intervals of the score
                
                for m = 1:size(best_scores,2)
                    n_dir = zeros(3,number_of_points(1,current_subset));
                    dist = zeros(1,number_of_points(1,current_subset));
                    acceptable_points = zeros(1,number_of_points(1,current_subset));
                    for i=1:number_of_points(1,current_subset)
                        [n_dir(:,i),dist(i)] = point_to_line(A_new_subset(1:3,i),[best_detected_cylinders(1:3,m) best_detected_cylinders(4:6,m)]);
                        if abs(dist(i)-best_detected_cylinders(7,m))<dist_tol &&...
                                abs(A_new_subset(4:6,i)'*n_dir(:,i))>angle_tol
                            acceptable_points(i) = 1;
                        end
                    end
                    
                    % adding acceptable points from the new subset to the total
                    % points for each candidate.
                    best_cylinders_points(m,id_new_subset(acceptable_points==1))=1;
                    if connectedness == 1
                        disp(['Size before LCC = ',num2str(sum(best_cylinders_points(m,:)))]);
                        best_cylinders_points(m,:) = extract_lcc(A(:,best_cylinders_points(m,:)==1),id(best_cylinders_points(m,:)==1),best_detected_cylinders(:,m),size_A,beta*sqrt(number_of_subsets/current_subset));
                        disp(['Size after LCC = ',num2str(sum(best_cylinders_points(m,:)))]);
                    end
                    best_scores(:,m) = score_estimation(size_A,number_of_points(2,current_subset),sum(best_cylinders_points(m,:)));
                end
                [best_scores,valid_cands] = sort_scores(best_scores);
                best_cylinders_points = best_cylinders_points(valid_cands,:);
                best_detected_cylinders = best_detected_cylinders(:,valid_cands);
                A_work = [A_work A_new_subset];
                id_work = [id_work id_new_subset];
            end
        end
        
    end
    % In the final iteration the shape with highest expected value is extracted
    if it==max_it
        disp('Max iterations reached')
        if size(best_detected_cylinders,2)==0 % in case no candidate was found
            % during the whole process,
            % best_detected_cylinders = []
            % as initially declared.
            disp('No shape detected');
            return;
        else
            extract_trigger =1;
        end
    end
    if extract_trigger ==1
        % searching unused subsets for compatible points
        best_detected_cylinder = best_detected_cylinders(:,1);
        rest_samples = sampling > current_subset/number_of_subsets;
        id_rest = id(:,rest_samples);
        A_rest = A(:,rest_samples);
        rest_size = size(A_rest,2);
        n_dir = zeros(3,rest_size);
        dist = zeros(1,rest_size);
        acceptable_points = zeros(1,rest_size);
        
        disp(['Extraction triggered. Evaluating the best candidate on the rest of the subsets. Rest size = ',num2str(rest_size)]);
        
        for i=1:rest_size
            [n_dir(:,i),dist(i)] = point_to_line(A_rest(1:3,i),[best_detected_cylinder(1:3) best_detected_cylinder(4:6)]);
            if abs(dist(i)-best_detected_cylinder(7))<dist_tol &&...
                    abs(A_rest(4:6,i)'*n_dir(:,i))>angle_tol
                acceptable_points(i) = 1;
            end
        end
        best_cylinder_points = best_cylinders_points(1,:);
        best_cylinder_points(id_rest(acceptable_points==1)) =1;
        if connectedness == 1
            disp(['Size before LCC (final) = ',num2str(sum(best_cylinder_points))]);
            best_cylinder_points = extract_lcc(A(:,best_cylinder_points==1),id(best_cylinder_points==1),best_detected_cylinder,size_A,beta);
            disp(['Size after LCC (final) = ',num2str(sum(best_cylinder_points))]);
            best_detected_cylinder = set_axis_length(A(:,best_cylinder_points==1),best_detected_cylinder);
        end
        disp(['Total size = ',num2str(sum(best_cylinder_points)),'(',num2str(sum(best_cylinders_points(1,:))),' work + ',num2str(sum(acceptable_points)),' rest)'])
        break
    end
end
%show_cylinder(best_detected_cylinder);

% refitting, if coresponding parameter is set.
if refitting ==1
    disp('Refitting initiated...');
    A_found = A(:,best_cylinder_points == 1);
    % A_rest = A(:,best_cylinder_points == 0);
    % Least squares starting vector x0 is a cylinder
    % found by RANSAC above
    b = best_detected_cylinder';
    x0 = [b(1:3) b(4:6)-b(1:3) b(7)];
    f = @(x)cyl_obj_fun(x,A_found);
    t_lsq = tic;
    [x,resnorm] = lsqnonlin(f,x0);
    disp(['Least squares refitting of ',num2str(size(A_found,2)),' points is complete! Time elapsed = ',num2str(toc(t_lsq))]);
    best_cylinder_points = zeros(1,size_A);
    n_dir = zeros(3,size_A);
    dist = zeros(1,size_A);
    for i=1:size_A
        [n_dir(:,i),dist(i)] = point_to_line(A(1:3,i),[x(1:3);x(1:3)+x(4:6)]');
        % The "scatter" is added to the shape only
        % AFTER the refitting, unlike what is suggested in the paper.
        if abs(dist(i)-x(7))<dist_tol*scatter_mult
            best_cylinder_points(i) = 1;
        end
    end
    
    
    best_detected_cylinder = [x(1:3) x(1:3)+x(4:6) x(7)]';
    
    
    if connectedness == 1
        disp(['Size before LCC (refitted) = ',num2str(sum(best_cylinder_points))]);
        
        best_cylinder_points = extract_lcc(A(:,best_cylinder_points==1),id(best_cylinder_points==1),best_detected_cylinder,size_A,beta);
        disp(['Size after LCC (refitted) = ',num2str(sum(best_cylinder_points))]);
        
        % best_detected_cylinder =
        set_axis_length(A(:,best_cylinder_points==1),best_detected_cylinder);
        
        %% Optional: check for size before output
        if sum(best_cylinder_points)>=min_points
            best_detected_cylinder = set_axis_length(A(:,best_cylinder_points==1),best_detected_cylinder);
        else
            best_detected_cylinder = zeros(7,1);
            best_cylinder_points = zeros(1,size_A);
        end
    end
end
disp(['RANSAC finished. Total time = ',num2str(toc(t_total))]);
end
function [tr_int] = score_estimation(total_size,subset_size,subset_score)
% The first and the third elements define a trust interval, the second element is
% the expected value.

N = -2 - subset_size;
x = -2 - total_size;
n = -1 - subset_score;
tr_int = -[1 1 1]'-(([1 1 1]'*x*n/N)+[-1 0 1]'*(sqrt(x*n*(N-x)*(N-n)/(N-1))/N));
end
function [new_scores,valid_cand_id] = sort_scores(scores)
s = size(scores,2);
A = [scores; zeros(1,s)];
for i = 1:s
    A(4,i) = i;
end
B = sortrows(A',-2);
B = B';
cand_id = B(4,:);
p = zeros(1,s);
for i= 1:s
    if B(3,i)>=B(1,1)
        p(i)= 1;
    end
end
valid_cand_id = cand_id(p==1);
new_scores = B(1:3,p==1);
end
function bool = not_found_yet(best_score,best_cyl,cand_score,cand_cyl)
bool = 1;
if (cand_score/best_score)>0.99
    if (cand_cyl(7)/best_cyl(7))>0.95 && (cand_cyl(7)/best_cyl(7))<1.05
        if abs(((best_cyl(4:6)-best_cyl(1:3))/norm((best_cyl(4:6)-best_cyl(1:3))))'*((cand_cyl(4:6)-cand_cyl(1:3))/norm((cand_cyl(4:6)-cand_cyl(1:3)))))>0.985 % scalar product of both axes gives us the cosine of the angle between them. 0.985 is a cosine of a 5 degree angle.
            bool = 0; % if all checks are passed, then the cylinders are very similar. The new candidate should be rejected to avoid having the same cylinder more than once among the candidates.
        end
    end
end
end


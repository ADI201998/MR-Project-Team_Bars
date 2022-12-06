function visualize(seqID, start_frm, end_frm, carID)

numFrames = end_frm - start_frm + 1;
seq = seqID .* ones(1, numFrames);
frm = start_frm:1:end_frm;
id = carID .* ones(1, numFrames);
numObs = 36;
numPts = 36;
numVecs = 42;
K = [721.53,0,609.55;0,721.53,172.85;0,0,1];
avgCarLength = 3.8600;
avgCarWidth = 1.6362;
avgCarHeight = 1.5208;
multi_opt_wireframe = importdata("files/ceres_output_multiViewAdjuster.txt");
% system("rm transLog.txt");
f = fopen("files/transLog.txt","w");
f1 = fopen("files/angleLog.txt","w");
for i=1:length(frm)
    image = "left_colour_imgs/" + string(seq(i)) + "_" + string(frm(i)) + ".png";
    wireframe = multi_opt_wireframe(36*i-35:36*i,:)';
    new_car_centers = mean(wireframe');
%     f = fopen("transLog.txt","a");
    fprintf(f, "%f %f %f\n", new_car_centers);
%     fclose(f);
    proj_wf = K * wireframe;
    wf_img = proj_wf(1:2,:) ./ proj_wf(3,:);
    img = figure;
    visualizeWireframe2D(image, wf_img);
    saveas(img, sprintf("multiViewResult/%d_%d_%d.png", seq(i), frm(i), id(i)));
    close(img);
    pause(0.1);
end
fclose(f);

% if seqID == 3 && carID == 1 && end_frm ~= 100 % hardcoding specifically for this case because keypoints are terrible
%     for i=end_frm:1
%         system("rm multiViewResult/3_" + string(i) + "_1.png");
%     end
% end

system("rm multiViewResult/" + string(seqID) + "_" + string(start_frm) + "_" + string(end_frm) + "_" + string(carID) + ".mp4");
system("ffmpeg -framerate 5 -start_number "+string(start_frm)+" -i 'multiViewResult/"+string(seqID)+"_%d_"+string(carID)+".png' -c:v libx264 -r 30  -vf 'pad=ceil(iw/2)*2:ceil(ih/2)*2' -pix_fmt yuv420p multiViewResult/" + string(seqID) + "_" + string(start_frm) + "_" + string(end_frm) + "_" + string(carID) + ".mp4");
final_rot = importdata("files/rotLog.txt");
final_pose = importdata("files/transLog.txt");
f = fopen(string(seqID) + "_" + string(start_frm) + "_" + string(end_frm) + "_" + string(carID) + ".txt","w");
for i=1:size(final_rot, 1)
    fprintf(f, "%f %f %f %f %f %f %f %f %f %f %f %f\n", final_rot(i,:), final_pose(i,:));
end
fclose(f);
end
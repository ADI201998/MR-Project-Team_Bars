#include <multiViewShapeAdjuster.h>
#include <random>
#include <bits/stdc++.h>

#define MAXBUFSIZE  ((int) 1e6)
Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

/*
##############################################################################################

##############################################################################################
*/
multiViewShapeAdjuster::multiViewShapeAdjuster(int seqID, int startFrm, int endFrm, int carID)
{
    this->seqID = seqID;
    this->startFrm = startFrm;
    this->endFrm = endFrm;
    this->carID = carID;

    numObs = 36;
    numPts = 36;
    numVecs = 42;
    numViews = 1;
    K = Eigen::ArrayXXd(3,3);
    K << 721.53, 0, 609.55,
        0, 721.53, 172.85,
        0, 0, 1;
    avgCarLength = 3.86;
    avgCarWidth = 1.6362;
    avgCarHeight = 1.5208;
}

/*
##############################################################################################

##############################################################################################
*/
multiViewShapeAdjuster::~multiViewShapeAdjuster()
{

}

/*
##############################################################################################

##############################################################################################
*/
Eigen::ArrayXXd multiViewShapeAdjuster::readFile(std::string fileName)
{
    int cols = 0, rows = 0;
    std::vector<std::vector<double>> buff;

    // Read numbers from file into buffer.
    std::ifstream infile;
    infile.open(fileName);
    while (! infile.eof())
    {
        std::string line;
        std::getline(infile, line);

        int temp_cols = 0;
        std::stringstream stream(line);
        std::vector<double> val_temp;
        while(! stream.eof())
        {
            double val;
            stream >> val;
            val_temp.push_back(val);
            temp_cols+=1;
        }
		if(infile.eof())
			break;
        buff.push_back(val_temp);
        cols = temp_cols;
    }
    infile.close();

    // Populate matrix with numbers.
    Eigen::ArrayXXd result(buff.size(),cols);
    for (int i = 0; i < buff.size(); i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[i][j];
    return result;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::readLabels(std::string filePath, int seq)
{
	char fname_c[9];
	sprintf(fname_c, "/%04d.txt", seq);
	std::string fname = fname_c;
	filePath.append(fname);

	std::ifstream infile;
    infile.open(filePath);
    while (! infile.eof())
    {
        std::string line;
        std::getline(infile, line);

        int temp_cols = 0;
        std::stringstream stream(line);
        multiViewShapeAdjuster::Objects object;
		stream >> object.frame;
		stream >> object.id;
		stream >> object.type;
		stream >> object.truncation;
		stream >> object.occlusion;
		stream >> object.alpha;
		stream >> object.x1;
		stream >> object.y1;
		stream >> object.x2;
		stream >> object.y2;
		stream >> object.h;
		stream >> object.w;
		stream >> object.l;
		stream >> object.t1;
		stream >> object.t2;
		stream >> object.t3;
		stream >> object.ry;
		if(! stream.eof())
			stream >> object.score;
		objects.push_back(object);
    }

    infile.close();
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::tracklets_helper(Eigen::ArrayXd seq, Eigen::ArrayXd frm, Eigen::ArrayXd id, Eigen::ArrayXXd *tracklets_op, Eigen::ArrayXXd *ground_truth)
{
    std::vector<std::vector<double>> tov, gtv;
    int j = 0;
	for(int i=0; i<objects.size(); i++)
	{
        if(j>=frm.rows())
            break;
		if (objects[i].id == id(j, 0) && (objects[i].frame == frm(j, 0)))
		{
            std::vector<double> to_vect{seq(j, 0), frm(j, 0), id(j, 0), objects[i].x1, objects[i].y1, objects[i].x2, objects[i].y2, objects[i].ry};
            tov.push_back(to_vect);

            std::vector<double> gt_vect{seq(j, 0), frm(j, 0), id(j, 0), objects[i].t1, objects[i].t2, objects[i].t3};
            gtv.push_back(gt_vect);
            j+=1;
		}
	}
    Eigen::ArrayXXd to(tov.size(), 8), gt(gtv.size(), 6);
    for (int i = 0; i < tov.size(); i++)
    {
        for (int j = 0; j < 8; j++)
        {
            to(i, j) = tov[i][j];
            if(j<6)
                gt(i, j) = gtv[i][j];
        }
    }

	*tracklets_op = to;
	*ground_truth = gt;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::keypointLocalizations(Eigen::ArrayXd seq, Eigen::ArrayXd frm, Eigen::ArrayXd id, Eigen::ArrayXXd *new_seq_frm_id, Eigen::ArrayXXd *tracklets_op, Eigen::ArrayXXd *ground_truth, Eigen::ArrayXXd *keypoints_collection, Eigen::ArrayXXd *wkps)
{
    std::string infofile = "../files/infofile.txt";
    Eigen::ArrayXXd info = readFile(infofile);

    std::vector<int> indices;
    std::vector<std::vector<int>> new_seq_frm_id_vec;
    for(int i=0; i<info.rows(); i++)
    {
        std::vector<int> info_vals;
        bool flag = true;
        for(int j=0; j<seq.rows(); j++)
        {
            if((info(i,1) == seq(j,0)) && (info(i,2) == frm(j,0)) && (info(i,3) == id(j,0)))
            {
                indices.push_back(i);
                info_vals.push_back(info(i,1));
                info_vals.push_back(info(i,2));
                info_vals.push_back(info(i,3));
                flag = true;
                break;
            }
            else
                flag = false;
        }
        if(flag)
            new_seq_frm_id_vec.push_back(info_vals);
    }
    
    Eigen::ArrayXXd mat(new_seq_frm_id_vec.size(), 3);
    for (int i = 0; i < new_seq_frm_id_vec.size(); i++)
    {
        mat(i, 0) = new_seq_frm_id_vec[i][0];
        mat(i, 1) = new_seq_frm_id_vec[i][1];
        mat(i, 2) = new_seq_frm_id_vec[i][2];
    }
    *new_seq_frm_id = mat;

    std::string results_KP = "../files/result_KP.txt";
    Eigen::ArrayXXd kp = readFile(results_KP);


    Eigen::ArrayXXd data(indices.size(), kp.cols());
    for(int i=0; i<indices.size(); i++)
        data.row(i) = kp.row(indices[i]);
    
    Eigen::ArrayXXd to, gt;
    tracklets_helper(seq, frm, id, &to, &gt);

    Eigen::ArrayXXd w(kp.cols()/3, data.rows()), kc(data.rows()*2, kp.cols()/3);
    for(int i=0; i<data.rows(); i++)
    {
        //Eigen::Map<Eigen::ArrayXXd> keypoints(data.row(i).data(), 3, numObs);
        Eigen ::ArrayXXd keypoints = data.row(i);
        keypoints.resize(3, numObs);
        keypoints.row(0) = keypoints.row(0)*abs(to(i, 3) - to(i, 5))/64 + to(i, 3);
        keypoints.row(1) = keypoints.row(1)*abs(to(i, 4) - to(i, 6))/64 + to(i, 4);
        kc.row(i*2) = keypoints.row(0);
        kc.row(i*2 + 1) = keypoints.row(1);
        w.col(i) = keypoints.row(2);
    }

    *tracklets_op = to;
    *ground_truth = gt;
    *keypoints_collection = kc;
    *wkps = w;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::mobili(Eigen::ArrayXXd tracklets_op, Eigen::ArrayXXd ground_truth, Eigen::ArrayXXd *B)
{
    Eigen::ArrayXXd b(tracklets_op.rows(), 6);
    Eigen::ArrayXd n(3);
    n << 0, -1, 0;
    Eigen::MatrixXd I(3,3), K_inv(3,3);
    I.setIdentity();
    //K_inv = (K.matrix()).householderQr().solve(I);
    K_inv = K.matrix().inverse().array();

    for(int i=0; i<tracklets_op.rows(); i++)
    {
        Eigen::ArrayXd a(3);
        a << (tracklets_op(i, 3) + tracklets_op(i, 5))/2, tracklets_op(i, 6), 1;
        Eigen::ArrayXd op = ((-avgCarHeight*K_inv*a.matrix())*(n.transpose().matrix()*K_inv*a.matrix()).inverse()).array();
        op(1) = op(1) + -avgCarHeight/2;
        op(2) = op(2) + avgCarLength/2;
        b(i, 0) = tracklets_op(i, 0);
        b(i, 1) = tracklets_op(i, 1);
        b(i, 2) = tracklets_op(i, 2);
        b.row(i).tail(3) = op;
    }
    *B = b;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::scaleWireframes(Eigen::ArrayXXd *wireframe_scaled, Eigen::ArrayXXd *transformed_deformation_vectors)
{
    std::string shapeFile = "../files/meanShape.txt";
    Eigen::ArrayXXd wireframe1 = readFile(shapeFile);
    Eigen::ArrayXXd wireframe = wireframe1.transpose();

    Eigen::ArrayXd length_vec = (wireframe.col(17) + wireframe.col(35))/2 - (wireframe.col(10) + wireframe.col(28))/2;
    Eigen::ArrayXd width_vec = wireframe.col(6) - wireframe.col(24);
    Eigen::ArrayXd height_vec = (wireframe.col(14) + wireframe.col(13) + wireframe.col(32) + wireframe.col(31))/4 - (wireframe.col(5) + wireframe.col(6) + wireframe.col(24) + wireframe.col(23))/4;

    double length = length_vec.matrix().lpNorm<2>();
    double width = width_vec.matrix().lpNorm<2>();
    double height = height_vec.matrix().lpNorm<2>();

    double length_scale = avgCarLength/length;
    double width_scale = avgCarWidth/width;
    double height_scale = avgCarHeight/height;

    Eigen::ArrayXXd ws(wireframe.rows(), wireframe.cols());
    ws.row(0) = wireframe.row(0)*length_scale;
    ws.row(1) = wireframe.row(1)*height_scale;
    ws.row(2) = wireframe.row(2)*width_scale;

    std::string deformationFile = "../files/vectors.txt";
    Eigen::ArrayXXd deformation_vectors = readFile(deformationFile);
    Eigen::ArrayXXd tdv(deformation_vectors.rows(), deformation_vectors.cols());
    for(int i=0; i<deformation_vectors.rows(); i++)
    {
        Eigen::ArrayXXd in = deformation_vectors.row(i);
        in.resize(3, numObs);
        in.row(0) = in.row(0)*length_scale;
        in.row(1) = in.row(1)*height_scale;
        in.row(2) = in.row(2)*width_scale;
        in.resize(1, 108);
        tdv.row(i) = in;
    }
    *wireframe_scaled = ws;
    *transformed_deformation_vectors = tdv;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::initialTransformations(Eigen::ArrayXXd *old_wireframe, Eigen::ArrayXXd *old_def_vectors)
{
    Eigen::ArrayXXd wireframe_scaled, deformation_vectors;
    scaleWireframes(&wireframe_scaled, &deformation_vectors);
    Eigen::ArrayXXd R(3, 3);
    R << 0, 0, 1,
        0, -1, 0,
        1, 0, 0;
    Eigen::ArrayXXd transformed_coords = (R.matrix()*wireframe_scaled.matrix()).array();
    Eigen::ArrayXXd transformed_deformation_vectors(deformation_vectors.rows(), deformation_vectors.cols());
    transformed_deformation_vectors = 0;
    for(int i=0; i<deformation_vectors.rows(); i++)
    {
        Eigen::ArrayXXd in = deformation_vectors.row(i);
        in.resize(3,numObs);
        in = (R.matrix()*in.matrix()).array();
        in.resize(1, 108);
        transformed_deformation_vectors.row(i) = in;
    }
    *old_wireframe = transformed_coords;
    *old_def_vectors = transformed_deformation_vectors;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::approxAlignWireFrame(Eigen::ArrayXXd tracklets_data, Eigen::ArrayXXd B, Eigen::ArrayXXd *wireframe_collection, Eigen::ArrayXXd *def_vectors_collection, Eigen::ArrayXXd *rotation_collection)
{
    Eigen::ArrayXXd old_wireframe, old_def_vectors;
    initialTransformations(&old_wireframe, &old_def_vectors);
    Eigen::ArrayXd ry = tracklets_data.col(7);
    Eigen::ArrayXXd random_arr(ry.rows(), ry.cols());
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    for (int i = 0; i < random_arr.rows(); i++)
        for (int j = 0; j < random_arr.cols(); j++)
            random_arr(i,j) = distribution(generator)/10;
    
    Eigen::ArrayXd phi = ry + M_PI_2 + random_arr + (M_PI*10*distribution(generator))/180;
    Eigen::ArrayXXd T_temp(B.rows(), 3);
    T_temp.col(0) = B.col(3);
    T_temp.col(1) = B.col(4);
    T_temp.col(2) = B.col(5);
    Eigen::ArrayXXd T = T_temp.transpose();

    Eigen::ArrayXXd wc(phi.rows()*T.rows(), old_wireframe.cols()), dvc(old_def_vectors.rows()*phi.rows(), old_def_vectors.cols()), rc(phi.rows()*3, 3);
    for(int i=0; i<phi.rows(); i++)
    {
        Eigen::ArrayXXd R_y(3,3);
        R_y << Eigen::cos(phi.row(i)), 0, Eigen::sin(phi.row(i)),
                0, 1, 0,
                -Eigen::sin(phi.row(i)), 0, Eigen::cos(phi.row(i));
        rc.row(i*3) = R_y.row(0);
        rc.row(i*3 + 1) = R_y.row(1);
        rc.row(i*3 + 2) = R_y.row(2);

        Eigen::ArrayXXd new_wireframe = (R_y.matrix()*old_wireframe.matrix()).array();
        Eigen::ArrayXXd new_wireframe_translated = new_wireframe.colwise() + T.col(i);
        //Eigen::ArrayXXd new_wireframe_temp = (K.matrix()*new_wireframe_translated.matrix()).array();
        wc.row(i*3) = new_wireframe_translated.row(0);
        wc.row(i*3 + 1) = new_wireframe_translated.row(1);
        wc.row(i*3 + 2) = new_wireframe_translated.row(2);

        for(int j=0; j<old_def_vectors.rows(); j++)
        {
            Eigen::ArrayXXd in = old_def_vectors.row(j);
            in.resize(3, numObs);
            Eigen::ArrayXXd out = (R_y.matrix()*in.matrix()).array();
            out.resize(1, 108);
            dvc.row(i*old_def_vectors.rows() + j) = out;
        }
    }
    *wireframe_collection = wc;
    *def_vectors_collection = dvc;
    *rotation_collection = rc;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::keypointsWeights(Eigen::ArrayXXd wkps, Eigen::ArrayXXd tracklets_data, Eigen::ArrayXXd *observation_weights)
{
    std::string kpFile = "../files/kpLookup_azimuth.txt";
    Eigen::ArrayXXd kp_lookup_temp = readFile(kpFile);
    Eigen::ArrayXXd kp_lookup = kp_lookup_temp.transpose();
    Eigen::ArrayXXd ry = tracklets_data.col(tracklets_data.cols()-1);
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    Eigen::ArrayXXd azimuth = (ry + M_PI_2)*180/M_PI + 10*distribution(generator);
    Eigen::ArrayXXd wkpl(kp_lookup.rows(), azimuth.rows());
    for(int i=0; i<azimuth.rows(); i++)
    {
        if(azimuth(i, 0) >= 1)
        {
            double den = kp_lookup.col(azimuth(i, 0)).sum();
            Eigen::ArrayXXd num = kp_lookup.col(azimuth(i, 0));
            wkpl.col(i) = num/den;
        }
        else
        {
            double azm = azimuth(i, 0);
            double den = kp_lookup.col(360 - abs(azm)).sum();
            Eigen::ArrayXXd num = kp_lookup.col(360 - abs(azm));
            wkpl.col(i) = num/den;
        }
    }
    Eigen::ArrayXXd w = 0.3*wkps + 0.7*wkpl;
    double min = 0.001;
    for(int i=0; i<w.rows(); i++)
    {
        for(int j=0; j<w.cols(); j++)
        {
            if(w(i, j)<min)
                w(i, j) = min;
        }
    }
    *observation_weights = w;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::poseOptimizer(Eigen::ArrayXd frm, Eigen::ArrayXXd wireframe, Eigen::ArrayXXd def_vectors, Eigen::ArrayXXd rot_y, Eigen::ArrayXXd observation_wts, Eigen::ArrayXXd keypoints_collection, Eigen::ArrayXXd B, Eigen::ArrayXXd lambdas, Eigen::ArrayXXd *rotation_collection, Eigen::ArrayXXd *translation_collection)
{
    std::cout<<"poseOptimizer\n";
    std::ofstream rotLog;
    rotLog.open("../files/rotLog.txt");

    std::ofstream svpa;
    Eigen::ArrayXXd rc(frm.rows(), 9), tc(frm.rows(), 3);

    for(int i=0; i<frm.rows(); i++)
    {
        svpa.open("../files/ceres_input_singleViewPoseAdjuster.txt");
        svpa << 1 << " " << numPts << " " << numObs << std::endl;
        svpa << std::setprecision(10)<<B(i, 3) << " " << std::setprecision(10)<<B(i, 4) << " " << std::setprecision(10)<<B(i, 5) << std::endl;
        svpa << avgCarHeight << " " << avgCarWidth << " " << avgCarLength << std::endl;
        svpa << std::setprecision(10)<<K(0, 0) << " " << std::setprecision(10)<<K(0, 1) << " " << std::setprecision(10)<<K(0, 2) << " " << std::setprecision(10)<<K(1, 0) << " " << std::setprecision(10)<<K(1, 1) << " " << std::setprecision(10)<<K(1, 2) << " " << std::setprecision(10)<<K(2, 0) << " " << std::setprecision(10)<<K(2, 1) << " " << std::setprecision(10)<<K(2, 2) << std::endl;
        for(int j=0; j<keypoints_collection.cols(); j++)
            svpa << std::setprecision(10)<<keypoints_collection(i*2, j) << " " << std::setprecision(10)<<keypoints_collection(i*2 + 1, j) << std::endl;
        for(int j=0; j<observation_wts.rows(); j++)
            svpa << std::setprecision(10)<<observation_wts(j, i) << std::endl;
        for(int j=0; j<wireframe.cols(); j++)
            svpa << std::setprecision(10)<<wireframe(i*3, j) << " " << std::setprecision(10)<<wireframe(i*3+1, j) << " " << std::setprecision(10)<<wireframe(i*3+2, j) << std::endl;  
        for(int j=0; j<numVecs; j++)
        {
            for (int k=0; k<def_vectors.cols()-1; k++)
            {
                svpa << std::setprecision(10)<<def_vectors(i*numVecs+j, k) << " ";
            }
            svpa << std::setprecision(10)<<def_vectors(i*numVecs+j, def_vectors.cols()-1) << std::endl;
        }
        for(int j=0; j<lambdas.cols()-1; j++)
            svpa << std::setprecision(10)<<lambdas(0, j) << " ";
        svpa << std::setprecision(10)<<lambdas(0, lambdas.cols()-1) << std::endl;
        svpa.close();
        //exit(0);

        singleViewPoseAdjuster poseAdjuster;
        poseAdjuster.solve_singleViewPoseAdjuster();
        //system("./singleViewPoseAdjuster");


        std::ifstream fin("../files/ceres_output_singleViewPoseAdjuster.txt");
        Eigen::ArrayXXd r(1,9);
        Eigen::ArrayXXd T(1,3);
        double r_val, t_val;

        fin >> r_val;
        r(0,0) = r_val;
        fin >> r_val;
        r(0,1) = r_val;
        fin >> r_val;
        r(0,2) = r_val;
        fin >> r_val;
        r(0,3) = r_val;
        fin >> r_val;
        r(0,4) = r_val;
        fin >> r_val;
        r(0,5) = r_val;
        fin >> r_val;
        r(0,6) = r_val;
        fin >> r_val;
        r(0,7) = r_val;
        fin >> r_val;
        r(0,8) = r_val;

        fin >> T(0, 0);
        fin >> T(0, 1);
        fin >> T(0, 2);

        Eigen::ArrayXXd R = r;
        R.resize(3, 3);

        Eigen::ArrayXXd init_rot = rot_y.block(3*i, 0, 3, rot_y.cols());
        rc.row(i) = r;
        tc.row(i) = T.row(0);

        Eigen::ArrayXXd new_rot = (R.matrix()*init_rot.matrix()).array();
        rotLog << std::setprecision(10)<<new_rot(0, 0) << " " << std::setprecision(10)<<new_rot(0, 1) << " " << std::setprecision(10)<<new_rot(0, 2) << " " << std::setprecision(10)<<new_rot(1, 0) << " " << std::setprecision(10)<<new_rot(1, 1) << " " << std::setprecision(10)<<new_rot(1, 2) << " " << std::setprecision(10)<<new_rot(2, 0) << " " << std::setprecision(10)<<new_rot(2, 1) << " " << std::setprecision(10)<<new_rot(2, 2) << std::endl;
    }
    //exit(0);
    rotLog.close();
    *rotation_collection = rc;
    *translation_collection = tc;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::shapeOptimizer(Eigen::ArrayXd frm, Eigen::ArrayXXd wireframe, Eigen::ArrayXXd def_vectors, Eigen::ArrayXXd rot_y, Eigen::ArrayXXd observation_wts, Eigen::ArrayXXd keypoints_collection, Eigen::ArrayXXd B, Eigen::ArrayXXd *def_vectors_collection, Eigen::ArrayXXd *rotation_collection, Eigen::ArrayXXd *translation_collection, Eigen::ArrayXXd *lambdas_collection)
{
    Eigen::ArrayXXd lambdas(1, numVecs);
    lambdas << 0.0208000000000000,0.00970000000000000,0.00720000000000000,0.00570000000000000,0.00470000000000000,0.00330000000000000,0.00210000000000000,0.00160000000000000,0.00100000000000000,0.000900000000000000,0.000800000000000000,0.000800000000000000,0.000700000000000000,0.000600000000000000,0.000500000000000000,0.000500000000000000,0.000400000000000000,0.000400000000000000,0.000400000000000000,0.000300000000000000,0.000300000000000000,0.000300000000000000,0.000300000000000000,0.000300000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000200000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000,0.000100000000000000;

    Eigen::ArrayXXd rot_coll, trans_coll;
    poseOptimizer(frm, wireframe, def_vectors, rot_y, observation_wts, keypoints_collection, B, lambdas, &rot_coll, &trans_coll);
    std::cout<<"shapeOptimizer\n";

    std::ofstream svsa;
    Eigen::ArrayXXd lc(frm.rows(), numVecs), dc(frm.rows()*numVecs, def_vectors.cols());

    for(int i=0; i<frm.rows(); i++)
    {
        svsa.open("../files/ceres_input_singleViewShapeAdjuster.txt");
        svsa << 1 << " " << numPts << " " << numObs << std::endl;
        svsa << std::setprecision(10)<<B(i, 3) << " " << std::setprecision(10)<<B(i, 4) << " " << std::setprecision(10)<<B(i, 5) << std::endl;
        svsa << avgCarHeight << " " << avgCarWidth << " " << avgCarLength << std::endl;
        svsa << std::setprecision(10)<<K(0, 0) << " " << std::setprecision(10)<<K(0, 1) << " " << std::setprecision(10)<<K(0, 2) << " " << std::setprecision(10)<<K(1, 0) << " " << std::setprecision(10)<<K(1, 1) << " " << std::setprecision(10)<<K(1, 2) << " " << std::setprecision(10)<<K(2, 0) << " " << std::setprecision(10)<<K(2, 1) << " " << std::setprecision(10)<<K(2, 2) << std::endl;
        for(int j=0; j<keypoints_collection.cols(); j++)
            svsa << std::setprecision(10)<<keypoints_collection(i*2, j) << " " << std::setprecision(10)<<keypoints_collection(i*2 + 1, j) << std::endl;
        for(int j=0; j<observation_wts.rows(); j++)
            svsa << std::setprecision(10)<<observation_wts(j, i) << std::endl;
        for(int j=0; j<wireframe.cols(); j++)
            svsa << std::setprecision(10)<<wireframe(i*3, j) << " " << std::setprecision(10)<<wireframe(i*3+1, j) << " " << std::setprecision(10)<<wireframe(i*3+2, j) << std::endl; 
        for(int j=0; j<numVecs; j++)
        {
            for (int k=0; k<def_vectors.cols()-1; k++)
            {
                svsa << std::setprecision(10)<<def_vectors(i*numVecs+j, k) << " ";
            }
            svsa << std::setprecision(10)<<def_vectors(i*numVecs+j, def_vectors.cols()-1) << std::endl;
        }
        for(int j=0; j<lambdas.cols()-1; j++)
            svsa << std::setprecision(10)<<lambdas(0, j) << " ";
        svsa << std::setprecision(10)<<lambdas(0, lambdas.cols()-1) << std::endl;
        svsa << std::setprecision(10)<<rot_coll(i, 0) << std::endl;
        svsa << std::endl;
        svsa << std::setprecision(10)<<rot_coll(i, 1) << std::endl;
        svsa << std::endl;
        svsa << std::setprecision(10)<<rot_coll(i, 2) << std::endl;
        svsa << std::endl;
        svsa << std::setprecision(10)<<rot_coll(i, 3) << std::endl;
        svsa << std::endl;
        svsa << std::setprecision(10)<<rot_coll(i, 4) << std::endl;
        svsa << std::endl;
        svsa << std::setprecision(10)<<rot_coll(i, 5) << std::endl;
        svsa << std::endl;
        svsa << std::setprecision(10)<<rot_coll(i, 6) << std::endl;
        svsa << std::endl;
        svsa << std::setprecision(10)<<rot_coll(i, 7) << std::endl;
        svsa << std::endl;
        svsa << std::setprecision(10)<<rot_coll(i, 8) << std::endl;
        svsa << std::setprecision(10)<<trans_coll(i, 0) << std::endl;
        svsa << std::setprecision(10)<<trans_coll(i, 1) << std::endl;
        svsa << std::setprecision(10)<<trans_coll(i, 2) << std::endl;
        svsa.close();

        singleViewShapeAdjuster shapeAdjuster;
        shapeAdjuster.solve_singleViewShapeAdjuster();
        //system("./singleViewShapeAdjuster");

        Eigen::ArrayXXd lvals = readFile("../files/lvals.txt");
        Eigen::ArrayXXd vvals = readFile("../files/vvals.txt");

        lc.row(i) = lvals.row(0);
        for (int j=0; j<vvals.rows(); j++)
            dc.row(i*numVecs + j) = vvals.row(j);
    }
    *lambdas_collection = lc;
    *def_vectors_collection = dc;
    *rotation_collection = rot_coll;
    *translation_collection = trans_coll;
}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::multiViewAdjuster_fn()
{
    int numFrames = this->endFrm - this->startFrm + 1;
    std::cout<<"numFrames = "<<numFrames<<std::endl;
    std::cout<<"seqID = "<<this->seqID<<std::endl;
    std::cout<<"startFrm = "<<this->startFrm<<std::endl;
    std::cout<<"endFrm = "<<this->endFrm<<std::endl;
    std::cout<<"carID = "<<this->carID<<std::endl;
    Eigen::ArrayXd seq(numFrames), id(numFrames);
    seq = seqID;
    Eigen::ArrayXd frm = Eigen::ArrayXd::LinSpaced(numFrames, this->startFrm, this->endFrm);
    id = this->carID;
    numViews = numFrames;

    std::string filePath = "../files/training/label_02";
	readLabels(filePath, seq(0,0));

    Eigen::ArrayXXd new_seq_frm_id;
    Eigen::ArrayXXd keypoints_collection, wkps, tracklets_op, ground_truth;
    std::cout<<"keypointLocalizations\n";
    keypointLocalizations(seq, frm, id, &new_seq_frm_id, &tracklets_op, &ground_truth, &keypoints_collection, &wkps);

    seq.col(0) = new_seq_frm_id.col(0);
    id.col(0) = new_seq_frm_id.col(2);
    frm.col(0) = new_seq_frm_id.col(1);

    Eigen::ArrayXXd B;
    std::cout<<"mobili\n";
    mobili(tracklets_op, ground_truth, &B);

    Eigen::ArrayXXd wireframe, def_vectors, rot_y;
    std::cout<<"approxAlignWireFrame\n";
    approxAlignWireFrame(tracklets_op, B, &wireframe, &def_vectors, &rot_y);
    

    Eigen::ArrayXXd observation_weights;
    std::cout<<"keypointsWeights\n";
    keypointsWeights(wkps, tracklets_op, &observation_weights);

    Eigen::ArrayXXd def_vectors_collection, rotation_collection, translation_collection, lambdas_collection;
    shapeOptimizer(frm, wireframe, def_vectors, rot_y, observation_weights, keypoints_collection, B, &def_vectors_collection, &rotation_collection, &translation_collection, &lambdas_collection);

    std::ofstream mva;
    mva.open("../files/ceres_input_multiViewAdjuster.txt");
    mva << numViews << " " << numPts << " " << numObs << " " << numVecs << std::endl;
    mva << avgCarHeight << " " << avgCarWidth << " " << avgCarLength << std::endl;
    mva << std::setprecision(10)<<K(0, 0) << " " << std::setprecision(10)<<K(0, 1) << " " << std::setprecision(10)<<K(0, 2) << " " << std::setprecision(10)<<K(1, 0) << " " << std::setprecision(10)<<K(1, 1) << " " << std::setprecision(10)<<K(1, 2) << " " << std::setprecision(10)<<K(2, 0) << " " << std::setprecision(10)<<K(2, 1) << " " << std::setprecision(10)<<K(2, 2) << std::endl;
    std::cout<<std::endl;
    std::cout<<"B = "<<B.rows()<<", "<<B.cols()<<std::endl;
    for(int i=0; i<numViews; i++)
        mva << std::setprecision(10)<<B(i, 3) << " " << std::setprecision(10)<<B(i, 4) << " " << std::setprecision(10)<<B(i, 5) << std::endl;
    
    std::cout<<"keypoints_collection = "<<keypoints_collection.rows()<<", "<<keypoints_collection.cols()<<std::endl;
    for(int i=0; i<numViews; i++)
        for(int j=0; j<keypoints_collection.cols(); j++)
            mva << std::setprecision(10)<<keypoints_collection(i*2, j) << " " << std::setprecision(10)<<keypoints_collection(i*2 + 1, j) << std::endl;
    
    std::cout<<"observation_weights = "<<observation_weights.rows()<<", "<<observation_weights.cols()<<std::endl;
    for(int i=0; i<numViews; i++)
        for(int j=0; j<observation_weights.rows(); j++)
            mva << std::setprecision(10)<<observation_weights(j, i) << std::endl;
    
    std::cout<<"wireframe = "<<wireframe.rows()<<", "<<wireframe.cols()<<std::endl;
    for(int i=0; i<numViews; i++)
        for(int j=0; j<wireframe.cols(); j++)
            mva << std::setprecision(10)<<wireframe(i*3, j) << " " << std::setprecision(10)<<wireframe(i*3+1, j) << " " << std::setprecision(10)<<wireframe(i*3+2, j) << std::endl; 

    std::cout<<"def_vectors = "<<def_vectors.rows()<<", "<<def_vectors.cols()<<std::endl;
    for(int i=0; i<numViews; i++)
    {
        for(int j=0; j<numVecs; j++)
        {
            for (int k=0; k<def_vectors.cols()-1; k++)
            {
                mva << std::setprecision(10)<<def_vectors(i*numVecs+j, k) << " ";
            }
            mva << std::setprecision(10)<<def_vectors(i*numVecs+j, def_vectors.cols()-1) << std::endl;
        }
    }

    std::cout<<"lambdas_collection = "<<lambdas_collection.rows()<<", "<<lambdas_collection.cols()<<std::endl;
    Eigen::ArrayXXd lambdas_mean = lambdas_collection.colwise().mean();
    std::cout<<lambdas_mean.rows()<<", "<<lambdas_mean.cols()<<std::endl;
    for(int i=0; i<lambdas_mean.cols(); i++)
        mva << std::setprecision(10)<<lambdas_mean.col(i) << " ";
    
    mva << "\n";

    std::cout<<"rotation_collection = "<<rotation_collection.rows()<<", "<<rotation_collection.cols()<<std::endl;
    for(int i=0; i<numViews; i++)
        for(int j=0; j<9; j++)
            mva << std::setprecision(10)<<rotation_collection(i,j) << std::endl;
    
    std::cout<<"translation_collection = "<<translation_collection.rows()<<", "<<translation_collection.cols()<<std::endl;
    for(int i=0; i<numViews; i++)
        for(int j=0; j<3; j++)
            mva << std::setprecision(10)<<translation_collection(i,j) << std::endl;    
    
    std::cout<<"multiViewAdjuster\n";
    multiViewAdjuster mVAdjuster;
    mVAdjuster.solve_multiViewAdjuster();
    //system("./multiViewAdjuster");
}
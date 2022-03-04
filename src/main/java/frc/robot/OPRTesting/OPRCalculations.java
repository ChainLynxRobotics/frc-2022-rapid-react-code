import java.lang.Math;
import java.util.ArrayList;
import java.util.AbstractCollection;
import java.util.Set;
import java.util.Iterator;
import java.util.HashSet;
import Jama.Matrix;
/*OPR calculator for 6 team matches (two 3-team alliances)
can either implement Cholesky decomposition or MMSE depending on type of dataset, where
the former is recommended for large datasets, use MMSE for smaller datasets
*/
public class DPRCalculations {
    public static int[] getTeams(int teamsPerAlliance, int[][][] playingTeams) {
        Set<Integer> sortedTeams = collections.synchronizedSortedSet( new TreeSet<>());
        for(int i = 0; i < playingTeams.length; i++) {
            for(int j = 0; j < teamsPerAlliance; j++) { //usually just 3 teams
                sortedTeams.set(playingTeams[i][0][j]); //red alliance in 0th
                sortedTeams.set(playingTeams[i][1][j]); //blue alliance in 1st
            }
        }
        int[] teamList = new int[playingTeams.size()];
        HashSet<Integer> teamHashSet = sortedTeams.iterator();
        int i = 0;
        while (teamHashSet.hasNext()) {
            teamList[i++] = teamHashSet.next();
        }
        //return array instead of list
        return teamList;
    }
    //lower triangular matrices
    public static Matrix getTriangular(Matrix matrix, int numRows, int numColumns) {
        matrix = new Matrix(numRows, numColumns);
        if (numRows != numColumns) {
            throw new IllegalArgumentException("numRows doesn't match numColumns");
        } else {
            for(int i = 0; i < numRows; i++) {
                for(int j = 0; j < numColumns; j++) {
                    if (j > i) {
                        matrix[i][j] = 0;
                    }
                }
            }
        }
    }
    //the more random the scores, the larger mmse is (generally between 0-3)
    public static double computeMMSE(Matrix Mr, Matrix Mb, int numScoredMatches) {
        //squared variance per alliance over squared variance per match
        double MrTotal = 0;
        double MbTotal = 0;
        for(int i = 1; i < numScoredMatches; i++) {
            //sum of total scores per alliance per match
            MrTotal += Mr.get(i, 1);
            MbTotal += Mb.get(i, 1);
        }
        double avMr = MrTotal/numScoredMatches;
        double avMb = MbTotal/numScoredMatches;
        //will be squared variance of scores for each alliance
        double MrPerAlliance = 0;
        double MbPerAlliance = 0;
        //will be squared variance of scores for each team on the respective alliances
        for(int i = 0; i < numScoredMatches; i++) {
            MrPerAlliance += Math.pow(avMr-Mr.get(i, 1), 2);
            MbPerAlliance += Math.pow(avMb-Mb.get(i, 1), 2);
        }
        MrPerAlliance = MrPerAlliance/numScoredMatches;
        MbPerAlliance = MbPerAlliance/numScoredMatches;
        double mmse = (MrPerAlliance + MbPerAlliance)/2;
        if (mmse > 3) {
            mmse = 3;
        } else if {mmse < 0) {
            mmse = 0;
        }
        return mmse;
    }
   //teamList[team number], scores[#match][red=0 or blue=1], playingTeams[#match][red=0 or blue=1][#teams per alliance]
    public static double[] computeOPRwithMMSE(double mmse, int[] teamList, int teamsPerAlliance, int[][][] playingTeams, int[][] scores) {
        ArrayList<Integer> TeamList = new ArrayList<>();
        int numTeams = teamList.length;
        for (int i = 0; i < numTeams; i++) {
            TeamList.add(teamList[i]);
        }
        int numScoredMatches = 0;
        for (int[] score : scores) {
            if(score > 0) {
                numScoredMatches++;
            }
        }
        //A matrices contain array of which teams are in which matches
        //M matrices contain array of alliance scores
        Matrix Ar = new Matrix(numScoredMatches, numTeams); //red alliance
        Matrix Ab = new Matrix(numScoredMatches, numTeams); //blue alliance
        Matrix Mr = new Matrix(numScoredMatches, 1);
        Matrix Mb = new Matrix(numScoredMatches, 1);
        Matrix Ao = new Matrix(2*numScoredMatches, numTeams);
        Matrix Mo = new Matrix(2*numScoredMatches, 1);
        int match;
        double scoreSum;
        for (int i = 0; i < scores.length; i++) {
            if(scores[i][0] > 0) {
                for(int j = 0; j < teamsPerAlliance; j++) {
                    //first index is the row, second is the column, last is value
                    Ar.set(match, Ar.getIndex(playingTeams[i][0][j]), 1);
                    Ab.set(match, Ab.getIndex(playingTeams[i][1][j]), 1);
                }
            //row, column, score
            Mr.set(match, 0, scores[i][0]);
            Mb.set(match, 1, scores[i][1]);
            }
            scoreSum += scores[i][0];
            scoreSum += scores[i][1];
            match++;
        }
            //set matrices (start and end row index, start and end column index, matrix used)
            //set Ao as a combination of submatrices Ar and Ab
            Ao.setMatrix(0, numScoredMatches -1, 0, numTeams-1, Ar);
            Ao.setMatrix(numScoredMatches, 2*numScoredMatches -1, 0, numTeams, Ab);
            double averageTeamOffenseScore = scoreSum/(2*numScoredMatches*numTeams);
            for (int i = 0; i < numScoredMatches; i++) {
                //adjust single robot's OPR by subtracting the other two teams' on alliance
                Mr.set(i, 0, Mr.get(i, 0) - 2*averageTeamOffenseScore);
                Mb.set(i, 0, Mb.get(i, 0) - 2*averageTeamOffenseScore);
            } 
            //set Mo as a combination of submatrices Mr and Mb
            Mo.setMatrix(0, numScoredMatches-1, 0, 0, Mr);
            Mo.setMatrix(numScoredMatches, 2*numScoredMatches-1, 0, 0, Mb);
            Matrix AoMatrixInverse;
            try{
                //Ao'Ao+mmse*identity matrix of Ao
                AoMatrixInverse = Ao.transpose().times(Ao).plus(MatrixIdentity(numTeams, numTeams).times(mmse)).inverse();
            } catch (Exception e) {
                return null;
            }
        double[] opr = new double[teamList.length];
        Matrix OPR = AoMatrixInverse.times(Ao.transpose().times(Mo));
        for (int i = 0; i < numTeams; i++) {
            OPR.set(i, 0, OPR.get(i, 0) + averageTeamOffenseScore);
            opr[i] = OPR.get(i, 0);
        }
        return opr;
    }
    //with Chloesky Decomposition
    public static double[] computeOPRwithChloesky(double mmse, int[] teamList, int teamsPerAlliance, int[][][] playingTeams, int[][] scores, double averageTeamOffenseScore) {
        computeOPRwithMMSE(mmse, teamList, teamsPerAlliance, playingTeams, scores);
        Matrix AoNew = getTriangular(Ao, 2*numScoredMatches, numTeams);
        Matrix MatrixInverseNew;
        try {
            AoMatrixInverseNew = AoNew.transpose().times(AoNew);
        } catch (Exception e) {
            return null;
        }
        double[] oprC = new double[teamList.length];
        Matrix OPRC = AoMatrixInverseNew.times(AoNew.transpose().times(Mo));
        for (int i = 0; i < numTeams; i++) {
            OPRC.set(i, 0, OPRC.get(i, 0) + averageTeamOffenseScore);
            oprC[i] = OPR.get(i, 0);
        }
        return oprC;
    }

    //returns data from computeOPR
    public static double[] data(double mmse, int[] teamList, int teamsPerAlliance, int[][][] playingTeams, int scores) {
        if (teamList.length < 1000) {
            computeOPRwithMMSE(mmse, teamList, teamsPerAlliance, playingTeams, scores);
        } else if (teamList.length >= 1000) {
            computeOPRwithChloesky(mmse, teamList, teamsPerAlliance, playingTeams, scores, averageTeamOffenseScore);
        }
    }
}

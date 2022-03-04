public class OPRProcessing {

    List<Match> matches = DataIO.getMatchData("ChainLynx.matches");

     /**
     * A simple data class representing the results of a frc match, implemented using lombok {@link lombok.Data @Data}.
     */
    @Data
    public class Match implements Serializable {
        /** Teams designated red that participated in the match. */
        public final int[] redTeamNumbers;
        /** The final score of the red team at the end of the match. */
        public final int redTeamScore;
        /** Teams designated blue that participated in the match. */
        public final int[] blueTeamNumbers;
        /** The final score of the blue team at the end of the match. */
        public final int blueTeamScore;
        /**
         * A boolean value representing the winning team.
         * True for red, false for blue.
         *  */
        public final boolean winningTeam;
        // TODO: score breakdown
    }
}

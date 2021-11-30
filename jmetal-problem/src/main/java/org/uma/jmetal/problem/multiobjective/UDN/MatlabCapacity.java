//https://es.mathworks.com/help/matlab/matlab_external/start-matlab-session-from-java.html
package org.uma.jmetal.problem.multiobjective.UDN;

//import com.mathworks.engine.*;


public class MatlabCapacity {
    public static void main(String[] args) throws Exception {
        //String[] myEngine = {"myMatlabEngine"}; 
        // MatlabEngine eng = MatlabEngine.startMatlab();
        String path = "MIMO";
        char[] rute = new char[path.length()];
        for (int i = 0; i < path.length(); i++) {
            rute[i] = path.charAt(i);
        }
        //eng.feval("cd",rute);
        //double[][] capacity= eng.feval("Calcula_Capacidad_para_PABLO");
//        for(int i=0; i<capacity.length; i++){
//            for(int j=0; j<capacity[0].length; j++){
//                System.out.print(capacity[i][j] + " ");
//            }
//            System.out.println("");
//        }


        //System.out.println("LONG: "+engines.length +"     " +engines);
//        MatlabEngine eng = MatlabEngine.connectMatlab(engines[0]);
//        eng.putVariable("prueba", 100);

    }
}

package top.symple.pathplanner;

import com.sun.net.httpserver.HttpServer;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.zip.ZipOutputStream;

public class SyncServer {
    public static void main(String[] args) throws IOException {
        String absolutePath = new File("./").getAbsolutePath();
        String path = absolutePath.substring(0, absolutePath.length() - 1) + "TeamCode\\src\\main\\deploy\\pathplanner";

        HttpServer httpServer = HttpServer.create(new InetSocketAddress(InetAddress.getLocalHost().getHostAddress(), 2525), 0);
        httpServer.createContext("/ping", httpExchange -> {
            String response = "Pong!";
            httpExchange.sendResponseHeaders(200, response.length());
            OutputStream os = httpExchange.getResponseBody();
            os.write(response.getBytes());
            os.close();
        });

        httpServer.createContext("/sync", httpExchange -> {
            File zipFile = zipFiles(getAllFiles());

            httpExchange.sendResponseHeaders(200, zipFile.length());
            httpExchange.getResponseHeaders().add("Content-Disposition", "attachment; filename=pathplanner.zip");
            httpExchange.setAttribute("Content-Type", "application/zip");

            OutputStream outputStream = httpExchange.getResponseBody();
            Files.copy(zipFile.toPath(), outputStream);
            outputStream.close();

            zipFile.delete();
        });

        httpServer.setExecutor(null);
        httpServer.start();

        System.out.println("Sync Server Started On: http:/" + httpServer.getAddress().toString() + "/");
    }

    public static File zipFiles(File[] files) throws IOException {
        String filePath = getProjectFolder() + "\\pathplanner.zip";
        ZipOutputStream outputStream = new ZipOutputStream(new FileOutputStream(filePath));

        for (File file : files) {
            FileInputStream in = new FileInputStream(file);

            outputStream.putNextEntry(new ZipEntry(file.getPath().replace(getDeployFolder() + "\\", "")));

            byte[] b = new byte[1024];
            int count;

            while ((count = in.read(b)) > 0) {
                outputStream.write(b, 0, count);
            }

            in.close();
        }

        outputStream.close();

        return new File(filePath);
    }

    public static File[] getAllFiles() {
        File deployDir = new File(getDeployFolder());
        if(!deployDir.exists() || !deployDir.isDirectory()) return new File[0];

        List<File> files = new ArrayList<>();
        listFiles(deployDir.getPath(), files);
        return files.toArray(File[]::new);
    }

    private static void listFiles(String directoryName, List<File> files) {
        File directory = new File(directoryName);

        // Get all files from a directory.
        File[] fList = directory.listFiles();
        if(fList != null) {
            for (File file : fList) {
                if (file.isFile()) {
                    files.add(file);
                } else if (file.isDirectory()) {
                    listFiles(file.getAbsolutePath(), files);
                }
            }
        }
    }

    public static String getDeployFolder() {
        String absolutePath = new File("./").getAbsolutePath();
        return absolutePath.substring(0, absolutePath.length() - 1) + "TeamCode\\src\\main\\deploy\\pathplanner";
    }

    public static String getProjectFolder() {
        String absolutePath = new File("./").getAbsolutePath();
        return absolutePath.substring(0, absolutePath.length() - 1) + "TeamCode\\pathplanner\\src\\main\\java\\top\\symple\\pathplanner";
    }
}
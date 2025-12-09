import http.requests.*;
import java.text.SimpleDateFormat;

public class WebClient {

  String url = "";
  String slackWebhook = "INPUT YOUR WEBHOOK";

  public WebClient() {
  }

  String getCurrentTime() {
    String pattern = "YYYY-MM-dd-hh-mm-ss";
    SimpleDateFormat simpleDateFormat = new SimpleDateFormat(pattern);
    String date = simpleDateFormat.format(new Date());
    return date;
  }

  // https://stackoverflow.com/questions/1359689/how-to-send-http-request-in-java
  // https://qiita.com/enkatsu/items/354a347ac9644acc2da7
  // https://github.com/runemadsen/HTTP-Requests-for-Processing
  // https://github.com/runemadsen/HTTP-Requests-for-Processing/blob/master/examples/jsonpost/jsonpost.pde
  public void postMessage(String msg) {
    PostRequest post = new PostRequest(slackWebhook);
    // post.addData("text", msg); // -> invalid_payload
    String newMsg = "[measurement_LSM6DS3tr: " + getCurrentTime() + "]\n" + msg;
    post.addData("{\"text\":\""+ newMsg + "\"}");
    post.send();
    println("Response Content: " + post.getContent());
    //println("Response Header: " + post.getHeader());
  }
}
